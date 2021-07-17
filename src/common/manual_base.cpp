//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_base.h"
namespace rm_manual {

ManualBase::ManualBase(ros::NodeHandle &nh) :
    data_(nh), nh_(nh),
    w_press_event_(boost::bind(&ManualBase::wPress, this, _1)) {
  controller_loader_ = new rm_common::ControllerLoader(nh);
  controller_loader_->loadControllers();
  calibration_manager_ = new rm_common::CalibrationManager(nh);
  ros::NodeHandle state_ctrl_nh(nh, "state_controllers_switch");
  switch_state_ctrl_srv_ = new rm_common::SwitchControllersService(state_ctrl_nh);
  switch_state_ctrl_srv_->startControllersOnly();
  switch_state_ctrl_srv_->callService();
  ros::NodeHandle base_ctrl_nh(nh, "base_controllers_switch");
  switch_base_ctrl_srv_ = new rm_common::SwitchControllersService(base_ctrl_nh);
}

void ManualBase::run() {
  ros::Time time = ros::Time::now();
  data_.referee_.read();
  checkReferee(time);
  calibration_manager_->checkCalibrate(time);
  checkSwitch(time);
  sendCommand(time);
}

void ManualBase::checkReferee(const ros::Time &time) {
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_
      && !data_.referee_.last_referee_data_.game_robot_status_.mains_power_chassis_output_) {
    ROS_INFO("Chassis output ON");
    chassisOutputOn();
  }
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_
      && !data_.referee_.last_referee_data_.game_robot_status_.mains_power_gimbal_output_) {
    ROS_INFO("Gimbal output ON");
    gimbalOutputOn();
  }
  if (data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_
      && !data_.referee_.last_referee_data_.game_robot_status_.mains_power_shooter_output_) {
    ROS_INFO("Shooter output ON");
    shooterOutputOn();
  }
}

void ManualBase::checkSwitch(const ros::Time &time) {
  if (remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() > 0.1) {
    ROS_INFO("Remote controller OFF");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() < 0.1) {
    ROS_INFO("Remote controller ON");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
//  if (last_switch_right_ != data_.dbus_data_.s_r) {
//    if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) rightSwitchUp();
//    else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) rightSwitchMid();
//    else if (data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) rightSwitchDown();
//  }
//  last_switch_right_ = data_.dbus_data_.s_r;

  if (state_ == RC)
    updateRc();
  else if (state_ == PC)
    updatePc();
}

void ManualBase::updateRc() {
//  if (last_switch_left_ != data_.dbus_data_.s_l) {
//    if (data_.dbus_data_.s_l == rm_msgs::DbusData::UP) leftSwitchUp();
//    else if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID) leftSwitchMid();
//    else if (data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN) leftSwitchDown();
//  }
//  last_switch_left_ = data_.dbus_data_.s_l;
}

void ManualBase::updatePc() {
  checkKeyboard();
  drawUi();
}

void ManualBase::checkKeyboard() {
  w_press_event_.update(data_.dbus_data_.key_w);
}

}



