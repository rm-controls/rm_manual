//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_manual.h"

namespace rm_manual {
ChassisGimbalManual::ChassisGimbalManual(ros::NodeHandle &nh) : ManualBase(nh) {
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, data_.referee_.referee_data_);
  ros::NodeHandle vel_nh(nh, "vel");
  vel_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
  ros::NodeHandle gimbal_nh(nh, "gimbal");
  gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_.referee_.referee_data_);
  ros::NodeHandle ui_nh(nh, "ui");
  state_ui_ = new StateUi(ui_nh, data_);
  armor_ui_ = new ArmorUi(ui_nh, data_);
  capacitor_ui_ = new DataUi(ui_nh, data_);
  warning_ui_ = new WarningUi(ui_nh, data_);
}

void ChassisGimbalManual::sendCommand(const ros::Time &time) {
  chassis_cmd_sender_->sendCommand(time);
  vel_cmd_sender_->sendCommand(time);
  gimbal_cmd_sender_->sendCommand(time);
}

void ChassisGimbalManual::updateRc() {
  ManualBase::updateRc();
  if (std::abs(data_.dbus_data_.wheel) > 0.01) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  } else
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setAngularZVel(data_.dbus_data_.wheel);
  vel_cmd_sender_->setLinearXVel(data_.dbus_data_.ch_r_y);
  vel_cmd_sender_->setLinearYVel(-data_.dbus_data_.ch_r_x);
  gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
}

void ChassisGimbalManual::updatePc() {
  ManualBase::updatePc();
  gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x, data_.dbus_data_.m_y);
}

void ChassisGimbalManual::rightSwitchDown(ros::Duration duration) {
  ManualBase::rightSwitchDown(duration);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchMid(ros::Duration duration) {
  ManualBase::rightSwitchMid(duration);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::rightSwitchUp(ros::Duration duration) {
  ManualBase::rightSwitchUp(duration);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  state_ui_->add();
  capacitor_ui_->add();
}

void ChassisGimbalManual::leftSwitchDown(ros::Duration duration) {
  ManualBase::leftSwitchDown(duration);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::gPress(ros::Duration /*duration*/) {
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.);
  } else {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    vel_cmd_sender_->setAngularZVel(1.);
  }
}

void ChassisGimbalManual::ePress(ros::Duration /*duration*/) {
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::TWIST)
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  else
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::TWIST);
}

void ChassisGimbalManual::drawUi(const ros::Time &time) {
  state_ui_->update("chassis", chassis_cmd_sender_->getMsg()->mode, chassis_cmd_sender_->getBurstMode());
  capacitor_ui_->update(time);
  armor_ui_->update(time);
  warning_ui_->update("spin", time,
                      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO
                          && vel_cmd_sender_->getMsg()->angular.z != 0.);
  ManualBase::drawUi(time);
}

}
