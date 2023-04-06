//
// Created by yuchen on 2023/4/3.
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

  is_balance_ = true;
  state_sub_ = balance_nh.subscribe<rm_msgs::BalanceState>("/state", 1, &BalanceManual::balanceStateCallback, this);
  c_event_.setDelayTriggered(boost::bind(&BalanceManual::stateNormalizeDelay, this), 1.0, true);
  //  w_event_.setDelayTriggered(boost::bind(&BalanceManual::stateNormalizeDelay, this), 0.5, true);
  //  s_event_.setDelayTriggered(boost::bind(&BalanceManual::stateNormalizeDelay, this), 0.5, true);
  //  a_event_.setDelayTriggered(boost::bind(&BalanceManual::stateNormalizeDelay, this), 0.5, true);
  //  d_event_.setDelayTriggered(boost::bind(&BalanceManual::stateNormalizeDelay, this), 0.5, true);
  ctrl_x_event_.setDelayTriggered(boost::bind(&BalanceManual::modeNormalizeDelay, this), 3.0, true);
  ctrl_x_event_.setRising(boost::bind(&BalanceManual::ctrlXPress, this));
}

void BalanceManual::sendCommand(const ros::Time& time)
{
  ChassisGimbalShooterCoverManual::sendCommand(time);
  balance_cmd_sender_->sendCommand(time);
}

void BalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  ctrl_x_event_.update(dbus_data->key_ctrl && dbus_data->key_x);
}

void BalanceManual::cPress()
{
  ChassisGimbalShooterCoverManual::cPress();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::wPress()
{
  ChassisGimbalShooterCoverManual::wPress();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::sPress()
{
  ChassisGimbalShooterCoverManual::sPress();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::aPress()
{
  ChassisGimbalShooterCoverManual::aPress();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::dPress()
{
  ChassisGimbalShooterCoverManual::dPress();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void BalanceManual::ctrlXPress()
{
  if (balance_cmd_sender_->getMsg()->data == rm_msgs::BalanceState::NORMAL)
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  else if (balance_cmd_sender_->getMsg()->data == rm_msgs::BalanceState::FALLEN)
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
}

void BalanceManual::balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg)
{
  if ((ros::Time::now() - msg->header.stamp).toSec() < 0.2)
  {
    if (std::abs(msg->theta) > 0.1)
    {
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    }
  }
}

void BalanceManual::stateNormalizeDelay()
{
  if (!shift_event_.getState())
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  ROS_INFO("state normalize");
}

void BalanceManual::modeNormalizeDelay()
{
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  ROS_INFO("mode normalize");
}
}  // namespace rm_manual
