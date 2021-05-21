//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_common.h"
namespace rm_manual {

Manual::Manual(ros::NodeHandle &nh) : nh_(nh) {
  data_.init(nh_);
  ros::NodeHandle ctrl_handle(nh, "controller_manager");
  controller_manager_ = new ControllerManager(ctrl_handle);
  controller_manager_->loadAllControllers();
  controller_manager_->startInformationControllers();
}

void Manual::run() {
  ros::Time time = ros::Time::now();
  data_.referee_->read();
  checkSwitch(time);
  checkKeyboard(time);
  sendCommand(time);
  drawUi();
}

void Manual::checkSwitch(const ros::Time &time) {
  if (is_dbus_receive_ && ((time - data_.dbus_data_.stamp).toSec() > 0.1)) {
    is_dbus_receive_ = false;
    this->remoteControlTurnOff();
  }
  if (!is_dbus_receive_ && ((time - data_.dbus_data_.stamp).toSec() < 0.1)) {
    is_dbus_receive_ = true;
    this->remoteControlTurnOn();
  }
  if (data_.dbus_data_.s_l == rm_msgs::DbusData::UP) this->leftSwitchUp();
  else if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID) this->leftSwitchMid();
  else if (data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN) this->leftSwitchDown();
  if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) this->rightSwitchUp();
  else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) this->rightSwitchMid();
  else if (data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) this->rightSwitchDown();
}

void Manual::checkKeyboard(const ros::Time &time) {
  if (data_.dbus_data_.key_a) this->aPress(); else last_release_a_ = time;
  if (data_.dbus_data_.key_b) this->bPress(); else last_release_b_ = time;
  if (data_.dbus_data_.key_c) this->cPress(); else last_release_c_ = time;
  if (data_.dbus_data_.key_d) this->dPress(); else last_release_d_ = time;
  if (data_.dbus_data_.key_e) this->ePress(); else last_release_e_ = time;
  if (data_.dbus_data_.key_f) this->fPress(); else last_release_f_ = time;
  if (data_.dbus_data_.key_g) this->gPress(); else last_release_g_ = time;
  if (data_.dbus_data_.key_q) this->qPress(); else last_release_q_ = time;
  if (data_.dbus_data_.key_r) this->rPress(); else last_release_r_ = time;
  if (data_.dbus_data_.key_s) this->sPress(); else last_release_s_ = time;
  if (data_.dbus_data_.key_w) this->wPress(); else last_release_w_ = time;
  if (data_.dbus_data_.key_v) this->vPress(); else last_release_v_ = time;
  if (data_.dbus_data_.key_x) this->xPress(); else last_release_x_ = time;
  if (data_.dbus_data_.key_z) this->zPress(); else last_release_z_ = time;
  if (data_.dbus_data_.key_shift) this->shiftPress(); else last_release_shift_ = time;
  if (data_.dbus_data_.p_l) this->mouseLeftPress(); else last_release_mouse_left_ = time;
  if (data_.dbus_data_.p_r) this->mouseRightPress(); else last_release_mouse_right_ = time;
  if (data_.dbus_data_.key_ctrl && data_.dbus_data_.key_z) this->ctrlZPress(); else last_release_ctrl_z_ = time;
  if (data_.dbus_data_.key_ctrl && data_.dbus_data_.key_w) this->ctrlWPress(); else last_release_w_ = time;
}

}

