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

protected:
  void cPress() override;
  void wPress() override;
  void sPress() override;
  void aPress() override;
  void dPress() override;
  void sendCommand(const ros::Time& time) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void ctrlXPress();
  void balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg);
  void stateNormalizeDelay();
  void modeNormalizeDelay();

  rm_common::BalanceCommandSender* balance_cmd_sender_{};

private:
  ros::Subscriber state_sub_;
  InputEvent ctrl_x_event_;
};
}  // namespace rm_manual
