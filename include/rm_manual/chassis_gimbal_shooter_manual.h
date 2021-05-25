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
    shooter_cmd_sender_ = new ShooterCommandSender(shooter_nh, *data_.referee_);
  }
 protected:
  void leftSwitchDown() override {
    ChassisGimbalManual::leftSwitchDown();
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void leftSwitchMid() override {
    rm_manual::ChassisGimbalManual::leftSwitchMid();
    if (state_ == RC)
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
  void leftSwitchUp() override {
    rm_manual::ChassisGimbalManual::leftSwitchUp();
    if (state_ == RC) {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_cmd_sender_->checkGimbalError(data_.gimbal_des_error_.error);
    }
  }
  void rightSwitchDown() override {
    ChassisGimbalManual::rightSwitchDown();
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void rightSwitchUp() override {
    ChassisGimbalManual::rightSwitchUp();
    shooter_cmd_sender_->setMode(pc_shooter_mode_);
    shooter_cmd_sender_->setBurst(shooter_burst_flag_);
    if (mouse_left_pressing_flag_ && ros::Time::now() - last_release_mouse_left_ < ros::Duration(0.02)) {
      mouse_left_pressing_flag_ = false;
      ui_->displayShooterInfo(pc_chassis_mode_, shooter_burst_flag_, graphic_operate_type_);
    }
  }
  void qPress() override {
    if (state_ == PC && ros::Time::now() - last_release_q_ < ros::Duration(0.02)) {
      shooter_burst_flag_ = !shooter_burst_flag_;
      ui_->displayShooterInfo(pc_shooter_mode_, shooter_burst_flag_, graphic_operate_type_);
    }
  }
  void mouseLeftPress() override {
    if (state_ == PC) {
      pc_shooter_mode_ = rm_msgs::ShootCmd::READY;
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_cmd_sender_->checkGimbalError(data_.gimbal_des_error_.error);
      if (!mouse_left_pressing_flag_) {
        mouse_left_pressing_flag_ = true;
        ui_->displayShooterInfo(rm_msgs::ShootCmd::PUSH, shooter_burst_flag_, graphic_operate_type_);
      }
    }
  }
  void ctrlWPress() override {
    ChassisGimbalManual::ctrlWPress();
    if (state_ == PC) ui_->displayShooterInfo(pc_shooter_mode_, shooter_burst_flag_, graphic_operate_type_);
  }
  void ctrlZPress() override {
    ChassisGimbalManual::ctrlZPress();
    pc_shooter_mode_ = rm_msgs::ShootCmd::STOP;
    shooter_burst_flag_ = false;
    ui_->displayShooterInfo(pc_shooter_mode_, shooter_burst_flag_, graphic_operate_type_);
  }
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
  }
  ShooterCommandSender *shooter_cmd_sender_;
  int pc_shooter_mode_ = rm_msgs::ShootCmd::STOP;
  bool shooter_burst_flag_ = false, mouse_left_pressing_flag_ = false;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
