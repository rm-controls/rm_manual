//
// Created by chenzheng on 7/20/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_COVER_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_COVER_MANUAL_H_
#include "rm_manual/chassis_gimbal_shooter_manual.h"

namespace rm_manual {
class ChassisGimbalShooterCoverManual : public ChassisGimbalShooterManual {
 public:
  explicit ChassisGimbalShooterCoverManual(ros::NodeHandle &nh);
  void run() override;
 protected:
  void checkKeyboard() override;
  void sendCommand(const ros::Time &time) override;
  void shooterOutputOn(ros::Duration duration) override;
  void rightSwitchDown(ros::Duration duration) override;
  void rightSwitchMid(ros::Duration duration) override;
  void rightSwitchUp(ros::Duration duration) override;
  void mouseRightPress(ros::Duration duration) override;
  void mouseRightRelease(ros::Duration duration) override;
  void ctrlZPress(ros::Duration /*duration*/);
  void drawUi(const ros::Time &time) override;
  rm_common::JointPositionBinaryCommandSender *cover_command_sender_{};
  rm_common::CalibrationQueue *cover_calibration_;
  RisingInputEvent ctrl_z_press_event_;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_COVER_MANUAL_H_
