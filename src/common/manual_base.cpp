//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_base.h"
namespace rm_manual
{
ManualBase::ManualBase(ros::NodeHandle& nh) : controller_manager_(nh), tf_listener_(tf_buffer_), nh_(nh)
{
  // sub
  joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &ManualBase::jointStateCallback, this);
  actuator_state_sub_ =
      nh.subscribe<rm_msgs::ActuatorState>("/actuator_states", 10, &ManualBase::actuatorStateCallback, this);
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &ManualBase::dbusDataCallback, this);
  track_sub_ = nh.subscribe<rm_msgs::TrackData>("/track", 10, &ManualBase::trackCallback, this);
  gimbal_des_error_sub_ = nh.subscribe<rm_msgs::GimbalDesError>("/controllers/gimbal_controller/error", 10,
                                                                &ManualBase::gimbalDesErrorCallback, this);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &ManualBase::odomCallback, this);
  game_robot_status_sub_ =
      nh.subscribe<rm_msgs::GameRobotStatus>("/game_robot_status", 10, &ManualBase::gameRobotStatusCallback, this);
  game_robot_hp_sub_ = nh.subscribe<rm_msgs::GameRobotHp>("/game_robot_hp", 10, &ManualBase::gameRobotHpCallback, this);
  game_status_sub_ = nh.subscribe<rm_msgs::GameStatus>("/game_status", 10, &ManualBase::gameStatusCallback, this);
  capacity_sub_ = nh.subscribe<rm_msgs::CapacityData>("/capacity_data", 10, &ManualBase::capacityDataCallback, this);
  power_heat_data_sub_ =
      nh.subscribe<rm_msgs::PowerHeatData>("/power_heat_data", 10, &ManualBase::powerHeatDataCallback, this);
  referee_sub_ = nh.subscribe<rm_msgs::Referee>("/referee", 10, &ManualBase::refereeCallback, this);
  // pub
  manual_to_referee_pub_ = nh.advertise<rm_msgs::ManualToReferee>("/manual_to_referee", 1);

  controller_manager_.startStateControllers();
  right_switch_down_event_.setRising(boost::bind(&ManualBase::rightSwitchDownRise, this));
  right_switch_mid_event_.setRising(boost::bind(&ManualBase::rightSwitchMidRise, this));
  right_switch_up_event_.setRising(boost::bind(&ManualBase::rightSwitchUpRise, this));
  left_switch_down_event_.setRising(boost::bind(&ManualBase::leftSwitchDownRise, this));
  left_switch_up_event_.setRising(boost::bind(&ManualBase::leftSwitchUpRise, this));
  left_switch_mid_event_.setEdge(boost::bind(&ManualBase::leftSwitchMidRise, this),
                                 boost::bind(&ManualBase::leftSwitchMidFall, this));
  robot_hp_event_.setEdge(boost::bind(&ManualBase::robotRevive, this), boost::bind(&ManualBase::robotDie, this));
}

void ManualBase::run()
{
  ros::Time time = ros::Time::now();
  checkReferee();
  checkSwitch(time);
  sendCommand(time);
  controller_manager_.update();
}

void ManualBase::checkReferee()
{
  robot_hp_event_.update(game_robot_status_data_.remain_hp != 0);
}

void ManualBase::checkSwitch(const ros::Time& time)
{
  if (remote_is_open_ && (time - dbus_data_.stamp).toSec() > 0.3)
  {
    ROS_INFO("Remote controller OFF");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - dbus_data_.stamp).toSec() < 0.3)
  {
    ROS_INFO("Remote controller ON");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
  right_switch_down_event_.update(dbus_data_.s_r == rm_msgs::DbusData::DOWN);
  right_switch_mid_event_.update(dbus_data_.s_r == rm_msgs::DbusData::MID);
  right_switch_up_event_.update(dbus_data_.s_r == rm_msgs::DbusData::UP);
  if (state_ == RC)
    updateRc();
  else if (state_ == PC)
    updatePc();
}

void ManualBase::updateRc()
{
  left_switch_down_event_.update(dbus_data_.s_l == rm_msgs::DbusData::DOWN);
  left_switch_mid_event_.update(dbus_data_.s_l == rm_msgs::DbusData::MID);
  left_switch_up_event_.update(dbus_data_.s_l == rm_msgs::DbusData::UP);
}

void ManualBase::updatePc()
{
  checkKeyboard();
}

void ManualBase::remoteControlTurnOff()
{
  controller_manager_.stopMainControllers();
  controller_manager_.stopCalibrationControllers();
  state_ = PASSIVE;
}

void ManualBase::remoteControlTurnOn()
{
  controller_manager_.startMainControllers();
  state_ = IDLE;
}

void ManualBase::robotRevive()
{
  if (remote_is_open_)
    controller_manager_.startMainControllers();
}

void ManualBase::robotDie()
{
  if (remote_is_open_)
  {
    controller_manager_.stopMainControllers();
    controller_manager_.stopCalibrationControllers();
  }
}

}  // namespace rm_manual
