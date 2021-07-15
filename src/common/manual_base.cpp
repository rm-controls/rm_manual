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
  if (last_switch_right_ != data_.dbus_data_.s_r) {
    if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) rightSwitchUp();
    else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) rightSwitchMid();
    else if (data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) rightSwitchDown();
  }
  last_switch_right_ = data_.dbus_data_.s_r;

  if (state_ == RC)
    updateRc();
  else if (state_ == PC)
    updatePc();
}

void ManualBase::updateRc() {
  if (last_switch_left_ != data_.dbus_data_.s_l) {
    if (data_.dbus_data_.s_l == rm_msgs::DbusData::UP) leftSwitchUp();
    else if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID) leftSwitchMid();
    else if (data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN) leftSwitchDown();
  }
  last_switch_left_ = data_.dbus_data_.s_l;
}

void ManualBase::updatePc() {
  checkKeyboard();
  drawUi();
}

void ManualBase::checkKeyboard() {
  if (data_.dbus_data_.key_w) wPress();
  else if (last_w_ && !data_.dbus_data_.key_w) wRelease();
  if (data_.dbus_data_.key_a) aPress();
  else if (last_a_ && !data_.dbus_data_.key_a) aRelease();
  if (data_.dbus_data_.key_s) sPress();
  else if (last_s_ && !data_.dbus_data_.key_s) sRelease();
  if (data_.dbus_data_.key_d) dPress();
  else if (last_d_ && !data_.dbus_data_.key_d) dRelease();
  if (!last_b_ && data_.dbus_data_.key_b) bPress();
  else if (last_b_ && !data_.dbus_data_.key_b) bRelease();
  if (!last_c_ && data_.dbus_data_.key_c) cPress();
  else if (last_c_ && !data_.dbus_data_.key_c) cRelease();
  if (!last_e_ && data_.dbus_data_.key_e) ePress();
  else if (last_e_ && !data_.dbus_data_.key_e) eRelease();
  if (!last_f_ && data_.dbus_data_.key_f) fPress();
  else if (last_f_ && !data_.dbus_data_.key_f) fRelease();
  if (!last_g_ && data_.dbus_data_.key_g) gPress();
  else if (last_g_ && !data_.dbus_data_.key_g) gRelease();
  if (!last_q_ && data_.dbus_data_.key_q) qPress();
  else if (last_q_ && !data_.dbus_data_.key_q) qRelease();
  if (!last_r_ && data_.dbus_data_.key_r) rPress();
  else if (last_r_ && !data_.dbus_data_.key_r) rRelease();
  if (!last_v_ && data_.dbus_data_.key_v) vPress();
  else if (last_v_ && !data_.dbus_data_.key_v) vRelease();
  if (!last_x_ && data_.dbus_data_.key_x) xPress();
  else if (last_x_ && !data_.dbus_data_.key_x) xRelease();
  if (!last_z_ && data_.dbus_data_.key_z) zPress();
  else if (last_z_ && !data_.dbus_data_.key_z) zRelease();
  if (!last_shift_ && data_.dbus_data_.key_shift) shiftPress();
  else if (last_shift_ && !data_.dbus_data_.key_shift) shiftRelease();
  if (!last_mouse_left_ && data_.dbus_data_.p_l) mouseLeftPress();
  else if (last_mouse_left_ && !data_.dbus_data_.p_l) mouseLeftRelease();
  if (!last_mouse_right_ && data_.dbus_data_.p_r) mouseRightPress();
  else if (last_mouse_right_ && !data_.dbus_data_.p_r) mouseRightRelease();
  if (!(last_ctrl_ & last_r_) && (data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r)) ctrlRPress();
  else if ((last_ctrl_ & last_r_) && !(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r)) ctrlRRelease();
  if (!(last_ctrl_ & last_v_) && (data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v)) ctrlVPress();
  else if ((last_ctrl_ & last_v_) && !(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v)) ctrlVRelease();
  if (!(last_ctrl_ & last_c_) && (data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c)) ctrlCPress();
  else if ((last_ctrl_ & last_c_) && !(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c)) ctrlCRelease();
  if (!(last_ctrl_ & last_z_) && (data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z)) ctrlZPress();
  else if ((last_ctrl_ & last_z_) && !(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z)) ctrlZRelease();

  last_a_ = data_.dbus_data_.key_a;
  last_b_ = data_.dbus_data_.key_b;
  last_c_ = data_.dbus_data_.key_c;
  last_d_ = data_.dbus_data_.key_d;
  last_e_ = data_.dbus_data_.key_e;
  last_f_ = data_.dbus_data_.key_f;
  last_g_ = data_.dbus_data_.key_g;
  last_q_ = data_.dbus_data_.key_q;
  last_r_ = data_.dbus_data_.key_r;
  last_s_ = data_.dbus_data_.key_s;
  last_w_ = data_.dbus_data_.key_w;
  last_v_ = data_.dbus_data_.key_v;
  last_x_ = data_.dbus_data_.key_x;
  last_z_ = data_.dbus_data_.key_z;
  last_shift_ = data_.dbus_data_.key_shift;
  last_mouse_left_ = data_.dbus_data_.p_l;
  last_mouse_right_ = data_.dbus_data_.p_r;
  last_ctrl_ = data_.dbus_data_.key_ctrl;
}

}

