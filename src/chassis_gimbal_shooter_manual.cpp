//
// Created by qiayuan on 5/22/21.
//

#include "rm_manual/chassis_gimbal_shooter_manual.h"
void rm_manual::ChassisGimbalShooterManual::leftSwitchDown() {
  ChassisGimbalManual::leftSwitchDown();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}
void rm_manual::ChassisGimbalShooterManual::leftSwitchMid() {
  rm_manual::ChassisGimbalManual::leftSwitchMid();
  if (state_ == RC) {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
}
void rm_manual::ChassisGimbalShooterManual::leftSwitchUp() {
  rm_manual::ChassisGimbalManual::leftSwitchUp();
  if (state_ == RC) {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    if (data_.gimbal_des_error_.error < gimbal_error_limit_)
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  }
}
