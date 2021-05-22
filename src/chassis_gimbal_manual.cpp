//
// Created by qiayuan on 5/22/21.
//
#include "rm_manual/chassis_gimbal_manual.h"
void rm_manual::ChassisGimbalManual::sendCommand(const ros::Time &time) {
  chassis_cmd_sender_->sendCommand(time);
  vel_cmd_sender_->sendCommand(time);
  gimbal_cmd_sender_->sendCommand(time);
}
