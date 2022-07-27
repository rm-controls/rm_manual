//
// Created by peter on 2020/12/3.
//

#ifndef RM_MANUAL_MANUAL_BASE_H_
#define RM_MANUAL_MANUAL_BASE_H_

#include "rm_manual/common/data.h"
#include "rm_manual/common/input_event.h"

#include <iostream>
#include <queue>
#include <tf/transform_listener.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <controller_manager_msgs/SwitchController.h>

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
  virtual void checkKeyboard(){};
  virtual void updateRc();
  virtual void updatePc();
  virtual void sendCommand(const ros::Time& time) = 0;

  virtual void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
  {
    data_.joint_state_ = *joint_state;
  }
  virtual void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
  {
    data_.actuator_state_ = *data;
  }
  virtual void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
  {
    data_.dbus_data_ = *data;
  }
  virtual void trackCallback(const rm_msgs::TrackData::ConstPtr& data)
  {
    data_.track_data_ = *data;
  }
  virtual void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data)
  {
    data_.gimbal_des_error_ = *data;
  }
  virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
  {
    data_.odom_ = *data;
  }

  virtual void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
  {
    data_.game_robot_status_data_ = *data;
  }
  virtual void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
  {
    data_.game_robot_hp_data_ = *data;
  }
  virtual void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
  {
    data_.game_status_data_ = *data;
  }
  virtual void capacityDataCallback(const rm_msgs::CapacityData ::ConstPtr& data)
  {
    data_.capacity_data_ = *data;
  }
  virtual void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
  {
    data_.power_heat_data_data_ = *data;
  }
  virtual void refereeCallback(const rm_msgs::Referee::ConstPtr& data)
  {
    data_.referee_sub_data_ = *data;
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

  Data data_;
  ros::NodeHandle nh_;
  rm_common::ControllerManager controller_manager_;

  bool remote_is_open_{};
  int state_ = PASSIVE;
  InputEvent robot_hp_event_, right_switch_down_event_, right_switch_mid_event_, right_switch_up_event_,
      left_switch_down_event_, left_switch_mid_event_, left_switch_up_event_;
};

}  // namespace rm_manual
#endif  // RM_MANUAL_MANUAL_BASE_H_
