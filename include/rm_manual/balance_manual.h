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
  void vPress();
  void gPress();
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
  void gyroCPressedCallback();

  rm_common::BalanceCommandSender* balance_cmd_sender_{};

private:
  ros::Subscriber state_sub_;

  double gyro_scale_;
  ros::Timer gyro_timer_;

  bool flank_ = false, dis_ = false;
  std::string flank_frame_, dis_frame_;
  InputEvent v_event_, g_event_, ctrl_x_event_;
};
}  // namespace rm_manual
