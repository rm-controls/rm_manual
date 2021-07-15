//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_base.h"
namespace rm_manual {

ManualBase::ManualBase(ros::NodeHandle &nh) : data_(nh), nh_(nh) {
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
  setZero();
  checkReferee(time);
  calibration_manager_->checkCalibrate(time);
  checkSwitch(time);
  checkKeyboard(time);
  sendCommand(time);
  drawUi();
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
  if (data_.dbus_data_.s_l == rm_msgs::DbusData::UP) leftSwitchUp();
  else if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID) leftSwitchMid();
  else if (data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN) leftSwitchDown();
  if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) rightSwitchUp();
  else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) rightSwitchMid();
  else if (data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) rightSwitchDown();
}

void ManualBase::checkKeyboard(const ros::Time &time) {
  if (data_.dbus_data_.key_a) aPress(); else last_release_a_ = time;
  if (data_.dbus_data_.key_b) bPress(); else last_release_b_ = time;
  if (data_.dbus_data_.key_c) cPress(); else last_release_c_ = time;
  if (data_.dbus_data_.key_d) dPress(); else last_release_d_ = time;
  if (data_.dbus_data_.key_e) ePress(); else last_release_e_ = time;
  if (data_.dbus_data_.key_f) fPress(); else last_release_f_ = time;
  if (data_.dbus_data_.key_g) gPress(); else last_release_g_ = time;
  if (data_.dbus_data_.key_q) qPress(); else last_release_q_ = time;
  if (data_.dbus_data_.key_r) rPress(); else last_release_r_ = time;
  if (data_.dbus_data_.key_s) sPress(); else last_release_s_ = time;
  if (data_.dbus_data_.key_w) wPress(); else last_release_w_ = time;
  if (data_.dbus_data_.key_v) vPress(); else last_release_v_ = time;
  if (data_.dbus_data_.key_x) xPress(); else last_release_x_ = time;
  if (data_.dbus_data_.key_z) zPress(); else last_release_z_ = time;
  if (data_.dbus_data_.key_shift) shiftPress(); else last_release_shift_ = time;
  if (data_.dbus_data_.p_l) mouseLeftPress(); else last_release_mouse_left_ = time;
  if (data_.dbus_data_.p_r) mouseRightPress(); else last_release_mouse_right_ = time;
  if (data_.dbus_data_.key_ctrl && data_.dbus_data_.key_r) ctrlRPress(); else last_release_ctrl_r_ = time;
  if (data_.dbus_data_.key_ctrl && data_.dbus_data_.key_v) ctrlVPress(); else last_release_ctrl_v_ = time;
  if (data_.dbus_data_.key_ctrl && data_.dbus_data_.key_c) ctrlCPress(); else last_release_ctrl_c_ = time;
  if (data_.dbus_data_.key_ctrl && data_.dbus_data_.key_z) ctrlZPress(); else last_release_ctrl_z_ = time;
}

}

