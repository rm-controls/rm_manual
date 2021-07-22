//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#include "rm_manual/chassis_gimbal_manual.h"
#include <rm_common/decision/calibration_queue.h>

namespace rm_manual {
class ChassisGimbalShooterManual : public ChassisGimbalManual {
 public:
  explicit ChassisGimbalShooterManual(ros::NodeHandle &nh);
  void run() override;
 protected:
  void checkKeyboard() override;
  void sendCommand(const ros::Time &time) override;
  void shooterOutputOn(ros::Duration /*duration*/) override;
  void updateRc() override;
  void rightSwitchDown(ros::Duration duration) override;
  void rightSwitchMid(ros::Duration duration) override;
  void rightSwitchUp(ros::Duration duration) override;
  void leftSwitchDown(ros::Duration duration) override;
  void leftSwitchMid(ros::Duration duration) override;
  void leftSwitchUp(ros::Duration duration) override;
  void mouseLeftPress(ros::Duration /*duration*/) override { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH); }
  void mouseLeftRelease(ros::Duration /*duration*/) override { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY); }
  void mouseRightPress(ros::Duration /*duration*/) override;
  void mouseRightRelease(ros::Duration /*duration*/) override { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
  void xPress(ros::Duration duration) override;
  void fPress(ros::Duration /*duration*/) { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void qPress(ros::Duration /*duration*/) { shooter_cmd_sender_->setBurstMode(!shooter_cmd_sender_->getBurstMode()); }
  void shiftPress(ros::Duration /*duration*/) { chassis_cmd_sender_->setBurstMode(true); }
  void shiftRelease(ros::Duration /*duration*/) { chassis_cmd_sender_->setBurstMode(false); }
  void ctrlCPress(ros::Duration /*duration*/) { gimbal_cmd_sender_->setBaseOnly(!gimbal_cmd_sender_->getBaseOnly()); }
  void ctrlVPress(ros::Duration /*duration*/);
  void ctrlRPress(ros::Duration /*duration*/);
  void ctrlBPress(ros::Duration /*duration*/);
  void drawUi() override;
  RisingInputEvent q_press_event_;
  RisingInputEvent f_press_event_;
  RisingInputEvent shift_press_event_;
  FallingInputEvent shift_release_event_;
  RisingInputEvent ctrl_c_press_event_;
  RisingInputEvent ctrl_v_press_event_;
  RisingInputEvent ctrl_r_press_event_;
  RisingInputEvent ctrl_b_press_event_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::SwitchDetectionCaller *switch_detection_srv_{};
  rm_common::CalibrationQueue *trigger_calibration_;
  AimUi *aim_ui_{};
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
