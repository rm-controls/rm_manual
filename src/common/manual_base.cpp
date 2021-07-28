//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_base.h"
namespace rm_manual {

ManualBase::ManualBase(ros::NodeHandle &nh) : data_(nh), nh_(nh), controller_manager_(nh) {
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

void ManualBase::run() {
  ros::Time time = ros::Time::now();
  data_.referee_.read();
  checkReferee();
  checkSwitch(time);
  sendCommand(time);
  drawUi(time);
  controller_manager_.update();
}

void ManualBase::checkReferee() {
  robot_hp_event_.update(data_.referee_.referee_data_.game_robot_status_.remain_hp_ != 0);
}

void ManualBase::checkSwitch(const ros::Time &time) {
  if (remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() > 0.3) {
    ROS_INFO("Remote controller OFF");
    remoteControlTurnOff();
    remote_is_open_ = false;
  }
  if (!remote_is_open_ && (time - data_.dbus_data_.stamp).toSec() < 0.3) {
    ROS_INFO("Remote controller ON");
    remoteControlTurnOn();
    remote_is_open_ = true;
  }
  right_switch_down_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN);
  right_switch_mid_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::MID);
  right_switch_up_event_.update(data_.dbus_data_.s_r == rm_msgs::DbusData::UP);
  if (state_ == RC)
    updateRc();
  else if (state_ == PC)
    updatePc();
}

void ManualBase::updateRc() {
  left_switch_down_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN);
  left_switch_mid_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::MID);
  left_switch_up_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
}

void ManualBase::updatePc() {
  checkKeyboard();
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

void ManualBase::robotRevive() {
  if (remote_is_open_) controller_manager_.startMainControllers();
}

void ManualBase::robotDie() {
  if (remote_is_open_) {
    controller_manager_.stopMainControllers();
    controller_manager_.stopCalibrationControllers();
  }
}

}



