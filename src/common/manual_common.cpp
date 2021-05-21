//
// Created by peter on 2020/12/3.
//

#include "rm_manual/common/manual_common.h"
namespace rm_manual {

Manual::Manual(ros::NodeHandle &node_handle) : nh_(node_handle) {
  data_.init(nh_);
  controller_manager_ = new ControllerManager(node_handle);
  controller_manager_->loadAllControllers();
  controller_manager_->startInformationControllers();
}

void Manual::run() {
  ros::Time time = ros::Time::now();

}

void Manual::checkSwitch(const ros::Time &time) {
  if (data_.referee_->is_open_) data_.referee_->read();
  if (is_rc_opened_ && ((time - data_.dbus_data_.stamp).toSec() > 0.1)) {
    is_rc_opened_ = false;
    this->remoteControlTurnOff();
  }
  if (!is_rc_opened_ && ((time - data_.dbus_data_.stamp).toSec() < 0.1)) {
    is_rc_opened_ = true;
    this->remoteControlTurnOn();
  }
  if (data_.dbus_data_.s_l == rm_msgs::DbusData::UP) this->leftSwitchUp();
  else if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID) this->leftSwitchMid();
  else if (data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN) this->leftSwitchDown();
  if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) this->rightSwitchUp();
  else if (data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) this->rightSwitchDown();
  else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) this->rightSwitchMid();
}

void Manual::checkKeyboard(const ros::Time &time) {
  if (data_.dbus_data_.key_f) this->fPress(time - last_press_f_);
  if (data_.dbus_data_.key_g) this->gPress(time - last_press_g_);
  if (data_.dbus_data_.key_r) this->sPress(time - last_press_s_);
  if (data_.dbus_data_.key_e) this->ePress(time - last_press_e_);
  if (data_.dbus_data_.key_r) this->rPress(time - last_press_r_);
  if (data_.dbus_data_.key_a) this->aPress(time - last_press_a_);
  if (data_.dbus_data_.key_d) this->dPress(time - last_press_d_);
  if (data_.dbus_data_.key_z) this->zPress(time - last_press_z_);
  if (data_.dbus_data_.key_x) this->xPress(time - last_press_x_);
  if (data_.dbus_data_.key_c) this->cPress(time - last_press_c_);
  if (data_.dbus_data_.key_v) this->vPress(time - last_press_v_);
  if (data_.dbus_data_.p_l && data_.dbus_data_.p_r) this->mouseLeftRightPress(time - last_press_mouse_right_left_);
  if (data_.dbus_data_.p_l) this->mouseRightPress(time - last_press_mouse_right_);
  if (data_.dbus_data_.p_r) this->mouseRightPress(time - last_press_mouse_right_);
  if (data_.dbus_data_.key_shift) this->shiftPress(time - last_press_shift_);
  if (data_.dbus_data_.key_q) {
    if (data_.dbus_data_.key_ctrl) this->ctrlQPress();
    else this->qPress(time - last_press_q_);
  }
  if (data_.dbus_data_.key_w) {
    if (data_.dbus_data_.key_ctrl) this->ctrlWPress();
    else this->wPress(time - last_press_w_);
  }
}

}
