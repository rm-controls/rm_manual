//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_base.h"
namespace rm_manual {

ManualBase::ManualBase(ros::NodeHandle &nh) : data_(nh), nh_(nh), controller_manager_(nh) {
  controller_manager_.startStateControllers();
  right_switch_down_rise_event_.setRising([this] { rightSwitchDownRise(); });
  right_switch_mid_rise_event_.setRising([this] { rightSwitchMidRise(); });
  right_switch_up_rise_event_.setRising([this] { rightSwitchUpRise(); });
  left_switch_down_rise_event_.setRising([this] { leftSwitchDownRise(); });
  left_switch_mid_rise_event_.setRising([this] { leftSwitchMidRise(); });
  left_switch_up_rise_event_.setRising([this] { leftSwitchUpRise(); });
}

void ManualBase::run() {
  ros::Time time = ros::Time::now();
  data_.referee_.read();
  checkReferee();
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
  right_switch_down_rise_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN);
  right_switch_mid_rise_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::MID);
  right_switch_up_rise_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::UP);
  if (state_ == RC)
    updateRc();
  else if (state_ == PC)
    updatePc();
}

void ManualBase::updateRc() {
  left_switch_down_rise_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN);
  left_switch_mid_rise_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::MID);
  left_switch_up_rise_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
}

void ManualBase::updatePc() {
  checkKeyboard();
  drawUi(ros::Time::now());
}

}



