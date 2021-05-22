//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#include "chassis_gimbal_manual.h"

namespace rm_manual {
class ChassisGimbalShooterManual : public ChassisGimbalManual {
 public:
  explicit ChassisGimbalShooterManual(ros::NodeHandle &nh) : ChassisGimbalManual(nh) {
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new ShooterCommandSender(shooter_nh, *data_.referee_);
  }
 protected:
  void leftSwitchDown() override {}
  void leftSwitchMid() override {}
  void leftSwitchUp() override {}
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
  }
  ShooterCommandSender *shooter_cmd_sender_;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
