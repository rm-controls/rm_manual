//
// Created by peter on 2020/12/3.
//

#pragma once

#include <queue>
#include <iostream>
#include <ros/ros.h>
#include <ros/timer.h>
#include <serial/serial.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <controller_manager_msgs/SwitchController.h>

#include <rm_common/ori_tool.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>

#include <rm_msgs/DbusData.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/CapacityData.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/ActuatorState.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/ManualToReferee.h>
#include "rm_manual/common/input_event.h"

namespace rm_manual
{
class ManualBase
{
public:
  explicit ManualBase(ros::NodeHandle& nh);
  enum
  {
    PASSIVE,
    IDLE,
    RC,
    PC
  };
  virtual void run();

protected:
  void checkSwitch(const ros::Time& time);
  virtual void checkReferee();
  virtual void checkKeyboard(const rm_msgs::DbusData::ConstPtr& data){};
  virtual void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data);
  virtual void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data);
  virtual void sendCommand(const ros::Time& time) = 0;

  virtual void jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
  {
    joint_state_ = *data;
  }
  virtual void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
  {
    dbus_data_ = *data;
    dbus_timer_.setPeriod(ros::Duration(0.3), true);
    if (!remote_is_open_)
    {
      ROS_INFO("Remote controller ON");
      remoteControlTurnOn();
      remote_is_open_ = true;
    }
    referee_is_online_ = (data->stamp - referee_last_get_ < ros::Duration(0.5));
    right_switch_down_event_.update(data->s_r == rm_msgs::DbusData::DOWN);
    right_switch_mid_event_.update(data->s_r == rm_msgs::DbusData::MID);
    right_switch_up_event_.update(data->s_r == rm_msgs::DbusData::UP);

    if (state_ == RC)
      updateRc(data);
    else if (state_ == PC)
      updatePc(data);
  }
  virtual void trackCallback(const rm_msgs::TrackData::ConstPtr& data)
  {
    track_data_ = *data;
  }
  virtual void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
  {
    robot_id_ = data->robot_id;
    robot_hp_event_.update(data->remain_hp != 0);
  }
  virtual void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
  {
    referee_last_get_ = data->stamp;
  }
  virtual void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data)
  {
    gimbal_des_error_ = *data;
  }
  virtual void capacityDataCallback(const rm_msgs::CapacityData ::ConstPtr& data)
  {
    chassis_power_ = data->chassis_power;
  }
  virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
  {
  }
  virtual void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
  {
  }
  virtual void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
  {
  }
  virtual void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
  {
  }
  virtual void dbusCloseCallback(const ros::TimerEvent& e)
  {
    if (remote_is_open_)
    {
      ROS_INFO("Remote controller OFF");
      remoteControlTurnOff();
      remote_is_open_ = false;
    }
  }

  // Referee
  virtual void chassisOutputOn()
  {
    ROS_INFO("Chassis output ON");
  }
  virtual void gimbalOutputOn()
  {
    ROS_INFO("Gimbal output ON");
  }
  virtual void shooterOutputOn()
  {
    ROS_INFO("Shooter output ON");
  }
  virtual void robotDie();
  virtual void robotRevive();

  // Remote Controller
  virtual void remoteControlTurnOff();
  virtual void remoteControlTurnOn();
  virtual void leftSwitchDownRise(){};
  virtual void leftSwitchMidRise(){};
  virtual void leftSwitchMidFall(){};
  virtual void leftSwitchUpRise(){};
  virtual void rightSwitchDownRise()
  {
    state_ = IDLE;
  }
  virtual void rightSwitchMidRise()
  {
    state_ = RC;
  }
  virtual void rightSwitchUpRise()
  {
    state_ = PC;
  }

  ros::Publisher manual_to_referee_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber referee_sub_;
  ros::Subscriber capacity_sub_;
  ros::Subscriber game_status_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber game_robot_hp_sub_;
  ros::Subscriber actuator_state_sub_;
  ros::Subscriber power_heat_data_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Subscriber game_robot_status_sub_;

  nav_msgs::Odometry odom_;
  sensor_msgs::JointState joint_state_;
  rm_msgs::DbusData dbus_data_;
  rm_msgs::TrackData track_data_;
  rm_msgs::GimbalDesError gimbal_des_error_;
  rm_msgs::ManualToReferee manual_to_referee_pub_data_;

  rm_common::ControllerManager controller_manager_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::NodeHandle nh_;

  ros::Time referee_last_get_;
  ros::Timer dbus_timer_;
  bool remote_is_open_{}, referee_is_online_ = false;
  int state_ = PASSIVE;
  int robot_id_, chassis_power_;
  InputEvent robot_hp_event_, right_switch_down_event_, right_switch_mid_event_, right_switch_up_event_,
      left_switch_down_event_, left_switch_mid_event_, left_switch_up_event_;
};

}  // namespace rm_manual
