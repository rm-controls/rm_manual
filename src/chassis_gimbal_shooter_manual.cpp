//
// Created by qiayuan on 5/22/21.
//

#include "rm_manual/chassis_gimbal_shooter_manual.h"
void rm_manual::ChassisGimbalShooterManual::leftSwitchDown() {
  ChassisGimbalManual::leftSwitchDown();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}
void rm_manual::ChassisGimbalShooterManual::leftSwitchMid() {
  ManualBase::leftSwitchMid();
  if (state_ == RC) {
    uint8_t target_id;
    target_id = gimbal_cmd_sender_->cost_function_->costFunction(data_.track_data_array_, false);
    shoot_speed_ = (int) data_.referee_->getShootSpeedLimit(shoot_speed_);
    if (target_id == 0) {
      if (last_target_id_ != 0) {
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
        gimbal_cmd_sender_->setId(last_target_id_);
        gimbal_cmd_sender_->setBulletSpeed(shoot_speed_);
      } else {
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
      }
    } else {
      last_target_id_ = target_id;
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->setId(target_id);
      gimbal_cmd_sender_->setBulletSpeed(shoot_speed_);
    }
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
}
void rm_manual::ChassisGimbalShooterManual::leftSwitchUp() {
  ManualBase::leftSwitchUp();
  if (state_ == RC) {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    uint8_t target_id;
    target_id = gimbal_cmd_sender_->cost_function_->costFunction(data_.track_data_array_, false);
    shoot_speed_ = (int) data_.referee_->getShootSpeedLimit(shoot_speed_);
    if (target_id == 0) {
      if (last_target_id_ != 0) {
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
        gimbal_cmd_sender_->setId(last_target_id_);
        gimbal_cmd_sender_->setBulletSpeed(shoot_speed_);
        if (data_.gimbal_des_error_.error < gimbal_error_limit_)
          shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      } else {
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
      }
    } else {
      last_target_id_ = target_id;
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->setId(target_id);
      gimbal_cmd_sender_->setBulletSpeed(shoot_speed_);
      if (data_.gimbal_des_error_.error < gimbal_error_limit_)
        shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    }

  }
}
