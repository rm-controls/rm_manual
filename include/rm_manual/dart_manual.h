//
// Created by luotinkai on 2022/7/15.
//

#pragma once

#include "rm_manual/manual_base.h"
#include <rm_common/decision/calibration_queue.h>
#include <rm_common/hardware_interface/gpio_interface.h>
#include <rm_msgs/GpioData.h>

namespace rm_manual
{
class DartManual : public ManualBase
{
public:
  DartManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void sendCommand(const ros::Time& time) override;
  void run() override;
  void checkReferee() override;
  void remoteControlTurnOn() override;
  void leftSwitchMidRise() override;
  void leftSwitchDownRise() override;
  void leftSwitchUpOn() override;

  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void rightSwitchUpRiseState() override
  {
    state_ = RC;
  }
  void rightSwitchDownOn() override;

  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void move(rm_common::JointPointCommandSender* joint, double ch);
  void recordPosition(const rm_msgs::DbusData::ConstPtr& dbus_data);

  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data);
  rm_common::JointPointCommandSender *trigger_sender_, *friction_left_sender_, *friction_right_sender_;
  rm_common::JointPointCommandSender *pitch_sender_, *yaw_sender_;
  rm_common::CalibrationQueue *trigger_calibration_, *gimbal_calibration_;
  double pitch_outpost_{}, pitch_base_{}, yaw_outpost_{}, yaw_base_{};
  double qd_, qd_1_, qd_2_, qd_3_, qd_4_, upward_vel_;
  double scale_{ 0.04 };
  bool if_stop_{ true };

  InputEvent chassis_power_on_event_, gimbal_power_on_event_;

  ros::Time start_;
  ros::Duration duration_ = ros::Duration(0.);
  ros::Duration upward_time_ = ros::Duration(2.2);
  bool flag_ = 0;
  int state_ = 1;
  rm_msgs::DbusData::ConstPtr data_;
  rm_control::GpioData gpio_state_;
  ros::Subscriber gpio_state_sub_;
  bool analog_level_ = 0, last_level_ = 0;
  bool door_state_2_ = false, door_state_3_ = false, door_state_4_ = false;
  bool return_state_1_ = false;
};
}  // namespace rm_manual
