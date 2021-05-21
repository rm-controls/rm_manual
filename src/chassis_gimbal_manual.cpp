//
// Created by qiayuan on 5/22/21.
//
#include "rm_manual/chassis_gimbal_manual.h"

void rm_manual::ChassisGimbalManual::rightSwitchMid() {
  chassis_command_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  ManualBase::rightSwitchMid();
}

void rm_manual::ChassisGimbalManual::rightSwitchUp() {
  ManualBase::rightSwitchUp();
}
