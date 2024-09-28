//
// Created by kook on 9/28/24.
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
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;

  void wPress() override;
  void sPress() override;
  void aPress() override;
  void dPress() override;
  void cPress() override;
  void shiftPress() override;
  void shiftRelease() override;
  void wPressing() override;
  void aPressing() override;
  void sPressing() override;
  void dPressing() override;
  void ctrlZPress() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void sendCommand(const ros::Time& time) override;

  rm_common::BalanceCommandSender* balance_cmd_sender_{};

private:
  bool flank_ = false, reverse_ = false;
  std::string flank_frame_, reverse_frame_;
};
}  // namespace rm_manual
