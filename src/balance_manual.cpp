//
// Created by kook on 9/28/24.
//

#include "rm_manual/balance_manual.h"

namespace rm_manual
{
BalanceManual::BalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalShooterCoverManual(nh, nh_referee)
{
  ros::NodeHandle balance_nh(nh, "balance");
  balance_cmd_sender_ = new rm_common::BalanceCommandSender(balance_nh);
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);

  nh.param("flank_frame", flank_frame_, std::string("flank_frame"));
  nh.param("reverse_frame", reverse_frame_, std::string("yaw_reverse_frame"));
  is_balance_ = true;
}

void BalanceManual::sendCommand(const ros::Time& time)
{
  if (flank_)
    chassis_cmd_sender_->getMsg()->follow_source_frame = flank_frame_;
  else if (reverse_)
    chassis_cmd_sender_->getMsg()->follow_source_frame = reverse_frame_;
  else
    chassis_cmd_sender_->getMsg()->follow_source_frame = "yaw";

  if (supply_)
  {
    cover_close_ = false;
    cover_command_sender_->on();
  }
  else
  {
    cover_close_ = true;
    cover_command_sender_->off();
  }

  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
  balance_cmd_sender_->sendCommand(time);
}

void BalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
}

void BalanceManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::updateRc(dbus_data);
  if (std::abs(dbus_data->ch_r_x) > 0.5 && std::abs(dbus_data->ch_r_x) > std::abs(dbus_data->ch_r_y))
    flank_ = true;
  else if (std::abs(dbus_data->ch_r_y) > 0.5 && std::abs(dbus_data->ch_r_y) > std::abs(dbus_data->ch_r_x))
    flank_ = false;
}

void BalanceManual::rightSwitchDownRise()
{
  ChassisGimbalShooterCoverManual::rightSwitchDownRise();
  state_ = RC;
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void BalanceManual::rightSwitchMidRise()
{
  ChassisGimbalShooterCoverManual::rightSwitchMidRise();
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::ctrlZPress()
{
  ChassisGimbalShooterCoverManual::ctrlZPress();
  if (supply_)
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  }
  else
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
}

void BalanceManual::shiftRelease()
{
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
}

void BalanceManual::shiftPress()
{
  ChassisGimbalShooterCoverManual::shiftPress();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::UP_SLOPE);
  chassis_cmd_sender_->updateSafetyPower(60);
}

void BalanceManual::wPress()
{
  if (flank_)
    flank_ = !flank_;
  if (!supply_)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  ChassisGimbalShooterCoverManual::wPress();
}

void BalanceManual::wPressing()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::wPressing();
  if (supply_)
    vel_cmd_sender_->setLinearXVel(x_scale_ * 0.4);
}

void BalanceManual::sPress()
{
  if (flank_)
    flank_ = !flank_;
  if (!supply_)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  ChassisGimbalShooterCoverManual::sPress();
}

void BalanceManual::sPressing()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::sPressing();
  if (supply_)
    vel_cmd_sender_->setLinearXVel(x_scale_ * 0.4);
}

void BalanceManual::aPress()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::aPress();
}

void BalanceManual::aPressing()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::aPressing();
  if (supply_)
    vel_cmd_sender_->setLinearYVel(y_scale_ * 0.4);
}

void BalanceManual::dPress()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::dPress();
}

void BalanceManual::dPressing()
{
  if (!flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::dPressing();
  if (supply_)
    vel_cmd_sender_->setLinearYVel(y_scale_ * 0.4);
}

void BalanceManual::cPress()
{
  if (is_gyro_)
  {
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  else
  {
    setChassisMode(rm_msgs::ChassisCmd::RAW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

}  // namespace rm_manual
