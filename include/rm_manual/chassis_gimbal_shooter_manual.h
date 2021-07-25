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
  void checkReferee() override;
  void checkKeyboard() override;
  void updateRc() override;
  void updatePc() override;
  void sendCommand(const ros::Time &time) override;
  void remoteControlTurnOff() override;
  void shooterOutputOn() override;
  void drawUi(const ros::Time &time) override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchDownRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchUpRise() override;
  void mouseLeftPress() override { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH); }
  void mouseLeftRelease() override { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY); }
  void mouseRightPress() override;
  void mouseRightRelease() override { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
  void xPress() override;
  void ePress();
  void gPress();
  void qPress() { shooter_cmd_sender_->setBurstMode(!shooter_cmd_sender_->getBurstMode()); }
  void fPress() { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void shiftPress() { chassis_cmd_sender_->setBurstMode(true); }
  void shiftRelease() { chassis_cmd_sender_->setBurstMode(false); }
  void ctrlCPress() { gimbal_cmd_sender_->setBaseOnly(!gimbal_cmd_sender_->getBaseOnly()); }
  void ctrlVPress();
  void ctrlRPress();
  void ctrlBPress();

  InputEvent shooter_power_on_event_, e_event_, g_event_, q_event_, f_event_, ctrl_c_event_, ctrl_v_event_,
      ctrl_r_event_, ctrl_b_event_, shift_event_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::SwitchDetectionCaller *switch_detection_srv_{};
  rm_common::CalibrationQueue *trigger_calibration_;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
