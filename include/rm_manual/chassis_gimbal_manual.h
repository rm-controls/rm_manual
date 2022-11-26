//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include "rm_manual/common/manual_base.h"

namespace rm_manual
{
class ChassisGimbalManual : public ManualBase
{
public:
  explicit ChassisGimbalManual(ros::NodeHandle& nh);

protected:
  void sendCommand(const ros::Time& time) override;
  void updateRc() override;
  void updatePc() override;
  void checkReferee() override;
  void checkKeyboard() override;
  void remoteControlTurnOff() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchMidFall() override;
  void leftSwitchDownRise() override;
  virtual void wPress()
  {
    x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  }
  virtual void wRelease();
  virtual void sPress()
  {
    x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  }
  virtual void sRelease();
  virtual void aPress()
  {
    y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  }
  virtual void aRelease();
  virtual void dPress()
  {
    y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  }
  virtual void dRelease();
  void wPressing();
  void aPressing();
  void sPressing();
  void dPressing();
  void mouseMidRise();

  rm_common::Vel2DCommandSender* vel_cmd_sender_{};
  rm_common::GimbalCommandSender* gimbal_cmd_sender_{};
  rm_common::ChassisCommandSender* chassis_cmd_sender_{};
  double x_scale_{}, y_scale_{};
  double gimbal_scale_{ 1. };
  double gyro_move_reduction_{ 1. };
  double gyro_rotate_reduction_{ 1. };

  bool turn_flag_ = false;

  InputEvent chassis_power_on_event_, gimbal_power_on_event_, w_event_, s_event_, a_event_, d_event_, mouse_mid_event_;
};
}  // namespace rm_manual
