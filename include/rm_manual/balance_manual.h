//
// Created by yuchen on 2023/4/3.
//

#pragma once

#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"

namespace rm_manual
{
class BalanceManual : public ChassisGimbalShooterCoverManual
{
public:
  BalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  enum ControlMethod
  {
    LQR,
    MPC
  };

protected:
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void wPress() override;
  void sPress() override;
  void aPress() override;
  void dPress() override;
  void shiftPress() override;
  void shiftRelease() override;
  void wPressing() override;
  void aPressing() override;
  void sPressing() override;
  void dPressing() override;
  void bPress() override;
  void ctrlZPress() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void remoteControlTurnOn() override;
  void remoteControlTurnOff() override;

  void sendCommand(const ros::Time& time) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void vPress();
  void ctrlXPress();
  void modeFallen(ros::Duration duration);
  void modeNormalize();
  rm_common::BalanceCommandSender* balance_cmd_sender_{};

private:
  void balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg);

  ros::Subscriber state_sub_;
  bool control_method_change_ = false;
  int control_method_ = ControlMethod::MPC;
  double balance_dangerous_angle_;
  std::vector<std::string> mpc_controllers_, lqr_controllers_;
  rm_common::JointPointCommandSender *left_momentum_block_cmd_sender_{}, *right_momentum_block_cmd_sender_{};

  bool flank_ = false, reverse_ = false;
  std::string flank_frame_, reverse_frame_;

  InputEvent v_event_, ctrl_x_event_, auto_fallen_event_;
};
}  // namespace rm_manual
