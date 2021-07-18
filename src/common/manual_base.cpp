//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_base.h"
namespace rm_manual {

ManualBase::ManualBase(ros::NodeHandle &nh) :
    data_(nh),
    nh_(nh),
    switch_right_down_event_(boost::bind(&ManualBase::rightSwitchDown, this, _1)),
    switch_right_mid_event_(boost::bind(&ManualBase::rightSwitchMid, this, _1)),
    switch_right_up_event_(boost::bind(&ManualBase::rightSwitchUp, this, _1)),
    switch_left_down_event_(boost::bind(&ManualBase::leftSwitchDown, this, _1)),
    switch_left_mid_event_(boost::bind(&ManualBase::leftSwitchMid, this, _1)),
    switch_left_up_event_(boost::bind(&ManualBase::leftSwitchUp, this, _1)),
    chassis_power_on_(boost::bind(&ManualBase::chassisOutputOn, this, _1)),
    gimbal_power_on_(boost::bind(&ManualBase::gimbalOutputOn, this, _1)),
    shooter_power_on_(boost::bind(&ManualBase::shooterOutputOn, this, _1)),
    w_press_event_(boost::bind(&ManualBase::wPress, this, _1)),
    w_release_event_(boost::bind(&ManualBase::wRelease, this, _1)),
    s_press_event_(boost::bind(&ManualBase::sPress, this, _1)),
    s_release_event_(boost::bind(&ManualBase::sRelease, this, _1)),
    a_press_event_(boost::bind(&ManualBase::aPress, this, _1)),
    a_release_event_(boost::bind(&ManualBase::aRelease, this, _1)),
    d_press_event_(boost::bind(&ManualBase::dPress, this, _1)),
    d_release_event_(boost::bind(&ManualBase::dRelease, this, _1)),
    mouse_left_press_event_(boost::bind(&ManualBase::mouseLeftPress, this, _1)),
    mouse_left_release_event_(boost::bind(&ManualBase::mouseLeftRelease, this, _1)),
    mouse_right_press_event_(boost::bind(&ManualBase::mouseRightPress, this, _1)),
    mouse_right_release_event_(boost::bind(&ManualBase::mouseRightRelease, this, _1)),
    x_press_event_(boost::bind(&ManualBase::xPress, this, _1)),
    x_release_event_(boost::bind(&ManualBase::xRelease, this, _1)),
    e_press_event_(boost::bind(&ManualBase::ePress, this, _1)),
    g_press_event_(boost::bind(&ManualBase::gPress, this, _1)) {
  controller_loader_ = new rm_common::ControllerManager(nh);
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
  chassis_power_on_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_);
  gimbal_power_on_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_);
  shooter_power_on_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_shooter_output_);
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
  switch_right_down_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN);
  switch_right_mid_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::MID);
  switch_right_up_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::UP);
  if (state_ == RC)
    updateRc();
  else if (state_ == PC)
    updatePc();
}

void ManualBase::updateRc() {
  switch_left_down_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN);
  switch_left_mid_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::MID);
  switch_left_up_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
}

void ManualBase::updatePc() {
  checkKeyboard();
  drawUi();
}

void ManualBase::checkKeyboard() {
  w_press_event_.update(data_.dbus_data_.key_w);
  w_release_event_.update(data_.dbus_data_.key_w);
  s_press_event_.update(data_.dbus_data_.key_s);
  s_release_event_.update(data_.dbus_data_.key_s);
  a_press_event_.update(data_.dbus_data_.key_a);
  a_release_event_.update(data_.dbus_data_.key_a);
  d_press_event_.update(data_.dbus_data_.key_d);
  d_release_event_.update(data_.dbus_data_.key_d);
  mouse_left_press_event_.update(data_.dbus_data_.p_l);
  mouse_left_release_event_.update(data_.dbus_data_.p_l);
  mouse_right_press_event_.update(data_.dbus_data_.p_r);
  mouse_right_release_event_.update(data_.dbus_data_.p_r);
  x_press_event_.update(data_.dbus_data_.key_x);
  x_release_event_.update(data_.dbus_data_.key_x);
  e_press_event_.update(data_.dbus_data_.key_e);
  g_press_event_.update(data_.dbus_data_.key_g);
}

}



