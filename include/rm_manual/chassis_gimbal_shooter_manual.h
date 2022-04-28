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
  void chassisOutputOn() override;
  void shooterOutputOn() override;
  void selfInspectionStart() { shooter_calibration_->reset(); };
  void gameStart() { shooter_calibration_->reset(); };
  void remoteControlTurnOff() override;
  void remoteControlTurnOn() override;
  void robotDie() override;
  void drawUi(const ros::Time &time) override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchDownRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchUpRise() override;
  void leftSwitchUpOn(ros::Duration duration);
  void mouseLeftPress();
  void mouseLeftRelease() { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY); }
  void mouseRightPress();
  void mouseRightRelease() { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
  void wPress() override;
  void aPress() override;
  void sPress() override;
  void dPress() override;
  void ePress();
  void gPress();
  void bPress();
  void qPress() { shooter_cmd_sender_->setBurstMode(!shooter_cmd_sender_->getBurstMode()); }
  void fPress() { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void shiftPress();
  void shiftRelease();
  void ctrlCPress();
  void ctrlVPress();
  void ctrlRPress();
  void ctrlBPress();

  InputEvent shooter_power_on_event_, self_inspection_event_, game_start_event_, e_event_, g_event_, q_event_, f_event_,
      b_event_, ctrl_c_event_, ctrl_v_event_, ctrl_r_event_, ctrl_b_event_, shift_event_, mouse_left_event_,
      mouse_right_event_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::SwitchDetectionCaller *switch_detection_srv_{};
  rm_common::CalibrationQueue *shooter_calibration_;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
