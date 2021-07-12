//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#include "rm_manual/chassis_gimbal_manual.h"

namespace rm_manual {
class ChassisGimbalShooterManual : public ChassisGimbalManual {
 public:
  explicit ChassisGimbalShooterManual(ros::NodeHandle &nh) : ChassisGimbalManual(nh) {
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_.referee_);
    ros::NodeHandle cover_nh(nh, "cover");
    cover_command_sender_ = new rm_common::CoverCommandSender(cover_nh);
    ui_shooter_ = new UiShooter(&data_.referee_);
  }
 protected:
  void drawUi() override {
    ChassisGimbalManual::drawUi();
    if (state_ == PC) ui_shooter_->display(shooter_cmd_sender_->getMsg()->mode, shooter_cmd_sender_->getBurstMode());
  }
  void setZero() override {
    ChassisGimbalManual::setZero();
    shooter_cmd_sender_->setZero();
  }
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
    cover_command_sender_->sendCommand(time);
  }
  void shooterOutputOn() override {
    ROS_INFO("Shooter Output on!");
    calibration_manager_->reset();
  }
  void leftSwitchDown() override {
    ChassisGimbalManual::leftSwitchDown();
    if (state_ == RC) shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void leftSwitchMid() override {
    rm_manual::ChassisGimbalManual::leftSwitchMid();
    if (state_ == RC) {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
      gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    }
  }
  void leftSwitchUp() override {
    rm_manual::ChassisGimbalManual::leftSwitchUp();
    if (state_ == RC) {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
    }
  }
  void rightSwitchDown() override {
    ChassisGimbalManual::rightSwitchDown();
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    cover_command_sender_->open();
  }
  void rightSwitchUp() override {
    ChassisGimbalManual::rightSwitchUp();
    if (shooter_cmd_sender_->getMsg()->mode == rm_msgs::ShootCmd::PUSH)
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    cover_command_sender_->close();
    ui_shooter_->setOperateType(UPDATE);
  }
  void rightSwitchMid() override {
    ChassisGimbalManual::rightSwitchMid();
    cover_command_sender_->close();
  }
  void fPress() override { if (state_ == PC) shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void qPress() override {
    if ((state_ == PC) && ros::Time::now() - last_release_q_ < ros::Duration(0.015))
      shooter_cmd_sender_->setBurstMode(!shooter_cmd_sender_->getBurstMode());
  }
  void xPress() override {
    ChassisGimbalManual::xPress();
    if (state_ == PC) ui_shooter_->setOperateType(ADD);
  }
  void mouseLeftPress() override {
    if (state_ == PC) { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH); }
  }
  void mouseRightPress() override {
    ChassisGimbalManual::mouseRightPress();
    if (state_ == PC) { gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed()); }
  }
  void ctrlWPress() override {
    if (state_ == IDLE) { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
    ChassisGimbalManual::ctrlWPress();
  }
  void ctrlZPress() override {
    if (state_ == PC) { shooter_cmd_sender_->setBurstMode(false); }
    ChassisGimbalManual::ctrlZPress();
  }
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::CoverCommandSender *cover_command_sender_{};
  UiShooter *ui_shooter_{};
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
