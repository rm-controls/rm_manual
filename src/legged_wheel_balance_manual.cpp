//
// Created by kook on 9/28/24.
//

#include "rm_manual/legged_wheel_balance_manual.h"

namespace rm_manual
{
LeggedWheelBalanceManual::LeggedWheelBalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : BalanceManual(nh, nh_referee)
{
  ros::NodeHandle leg_nh(nh, "balance/legged_wheel_chassis");
  legCommandSender_ = new rm_common::LegCommandSender(leg_nh);
  legCommandSender_->setLgeLength(0.18);
  legCommandSender_->setJump(false);

  b_event_.setEdge(boost::bind(&LeggedWheelBalanceManual::bPress, this),
                   boost::bind(&LeggedWheelBalanceManual::bRelease, this));
  ctrl_shift_event_.setEdge(boost::bind(&LeggedWheelBalanceManual::ctrlShiftPress, this),
                            boost::bind(&LeggedWheelBalanceManual::ctrlShiftRelease, this));
  ctrl_g_event_.setRising(boost::bind(&LeggedWheelBalanceManual::ctrlGPress, this));
}

void LeggedWheelBalanceManual::sendCommand(const ros::Time& time)
{
  BalanceManual::sendCommand(time);
  legCommandSender_->sendCommand(time);
}

void LeggedWheelBalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  ctrl_g_event_.update(dbus_data->key_ctrl && dbus_data->key_g);
  ctrl_shift_event_.update(dbus_data->key_ctrl && dbus_data->key_shift);
}

void LeggedWheelBalanceManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  BalanceManual::updateRc(dbus_data);
  if (!is_gyro_)
  {  // Capacitor enter fast charge when chassis stop.
    if (!dbus_data->wheel && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW &&
        std::sqrt(std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) >
            0.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    else if (chassis_power_ < 6.0 && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
  else
  {
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void LeggedWheelBalanceManual::rightSwitchDownRise()
{
  BalanceManual::rightSwitchDownRise();
}

void LeggedWheelBalanceManual::rightSwitchMidRise()
{
  BalanceManual::rightSwitchMidRise();
}

void LeggedWheelBalanceManual::ctrlZPress()
{
  BalanceManual::ctrlZPress();
  if (!supply_)
  {
    is_gyro_ = false;
    legCommandSender_->setLgeLength(0.18);
  }
}

void LeggedWheelBalanceManual::shiftRelease()
{
  BalanceManual::shiftRelease();
}

void LeggedWheelBalanceManual::shiftPress()
{
  BalanceManual::shiftPress();
}

void LeggedWheelBalanceManual::ctrlShiftPress()
{
  legCommandSender_->setJump(true);
}

void LeggedWheelBalanceManual::ctrlShiftRelease()
{
  legCommandSender_->setJump(false);
}

void LeggedWheelBalanceManual::bPress()
{
  ChassisGimbalShooterCoverManual::bPress();
  chassis_cmd_sender_->updateSafetyPower(60);
}

void LeggedWheelBalanceManual::bRelease()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  chassis_cmd_sender_->updateSafetyPower(60);
}

void LeggedWheelBalanceManual::ctrlGPress()
{
  if (!stretch_)
  {
    legCommandSender_->setLgeLength(0.25);
    stretch_ = true;
  }
  else
  {
    legCommandSender_->setLgeLength(0.18);
    stretch_ = false;
  }
}
}  // namespace rm_manual
