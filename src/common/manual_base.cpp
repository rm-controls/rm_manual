//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_base.h"
namespace rm_manual {

ManualBase::ManualBase(ros::NodeHandle &nh) : data_(nh), nh_(nh), controller_manager_(nh) {
  controller_manager_.startStateControllers();
  switch_right_down_event_.setRising([this] { rightSwitchDown(); });
  switch_right_mid_event_.setRising([this] { rightSwitchMid(); });
  switch_right_up_event_.setRising([this] { rightSwitchUp(); });
  switch_left_down_event_.setRising([this] { leftSwitchDown(); });
  switch_left_mid_event_.setRising([this] { leftSwitchMid(); });
  switch_left_up_event_.setRising([this] { leftSwitchUp(); });
  chassis_power_on_.setRising([this] { chassisOutputOn(); });
  gimbal_power_on_.setRising([this] { gimbalOutputOn(); });
  shooter_power_on_.setRising([this] { shooterOutputOn(); });
  x_event_.setRising([this] { xPress(); });
  w_event_.setEdge([this] { wPress(); }, [this] { wRelease(); });
  s_event_.setEdge([this] { sPress(); }, [this] { sRelease(); });
  a_event_.setEdge([this] { aPress(); }, [this] { aRelease(); });
  d_event_.setEdge([this] { dPress(); }, [this] { dRelease(); });
  mouse_left_event_.setEdge([this] { mouseLeftPress(); }, [this] { mouseLeftRelease(); });
  mouse_right_event_.setEdge([this] { mouseRightPress(); }, [this] { mouseRightRelease(); });
}

void ManualBase::run() {
  ros::Time time = ros::Time::now();
  data_.referee_.read();
  checkReferee(time);
  checkSwitch(time);
  sendCommand(time);
  controller_manager_.update();
}

void ManualBase::remoteControlTurnOff() {
  controller_manager_.stopMainControllers();
  controller_manager_.stopCalibrationControllers();
  state_ = PASSIVE;
}

void ManualBase::remoteControlTurnOn() {
  controller_manager_.startMainControllers();
  state_ = IDLE;
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
  w_event_.update(data_.dbus_data_.key_w);
  s_event_.update(data_.dbus_data_.key_s);
  a_event_.update(data_.dbus_data_.key_a);
  d_event_.update(data_.dbus_data_.key_d);
  mouse_left_event_.update(data_.dbus_data_.p_l);
  mouse_right_event_.update(data_.dbus_data_.p_r);
  x_event_.update(data_.dbus_data_.key_x);
}

}



