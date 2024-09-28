//
// Created by yuchen on 2023/4/3.
//

#pragma once

#include "rm_manual/balance_manual.h"

namespace rm_manual
{
class WheeledBalanceManual : public BalanceManual
{
public:
  WheeledBalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void sendCommand(const ros::Time& time) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void ctrlZPress() override;
  void bPress() override;
  void vPress() override;
  void ctrlXPress();
  void modeFallen(ros::Duration duration);
  void modeNormalize();
  rm_common::BalanceCommandSender* balance_chassis_cmd_sender_{};

private:
  void balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg);

  ros::Subscriber state_sub_;
  double balance_dangerous_angle_;

  InputEvent v_event_, ctrl_x_event_, auto_fallen_event_;
};
}  // namespace rm_manual
