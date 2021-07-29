//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_

#include "rm_manual/common/manual_base.h"
namespace rm_manual {
class ChassisGimbalManual : public ManualBase {
 public:
  explicit ChassisGimbalManual(ros::NodeHandle &nh);
 protected:
  void sendCommand(const ros::Time &time) override;
  void updateRc() override;
  void updatePc() override;
  void checkReferee() override;
  void checkKeyboard() override;
  void drawUi(const ros::Time &time) override;
  void remoteControlTurnOff() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchMidFall() override;
  void leftSwitchDownRise() override;
  virtual void wPress();
  virtual void wRelease();
  virtual void aPress();
  virtual void aRelease();
  virtual void sPress();
  virtual void sRelease();
  virtual void dPress();
  virtual void dRelease();

  rm_common::ChassisCommandSender *chassis_cmd_sender_{};
  rm_common::Vel2DCommandSender *vel_cmd_sender_;
  rm_common::GimbalCommandSender *gimbal_cmd_sender_{};
  TimeChangeUi *time_change_ui_{};
  FlashUi *flash_ui_{};
  TriggerChangeUi *trigger_change_ui_{};
  FixedUi *fixed_ui_{};
  double x_scale_{}, y_scale_{};
  double gyro_move_reduction_{};

  InputEvent chassis_power_on_event_, gimbal_power_on_event_, w_event_, s_event_, a_event_, d_event_;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
