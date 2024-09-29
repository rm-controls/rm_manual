//
// Created by yuchen on 2023/4/3.
//

#include "rm_manual/wheeled_balance_manual.h"

namespace rm_manual
{
WheeledBalanceManual::WheeledBalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : BalanceManual(nh, nh_referee)
{
  ros::NodeHandle balance_chassis_nh(nh, "balance/wheeled_chassis");
  balance_chassis_cmd_sender_ = new rm_common::BalanceCommandSender(balance_chassis_nh);
  balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);

  balance_chassis_nh.param("balance_dangerous_angle", balance_dangerous_angle_, 0.3);

  state_sub_ = balance_chassis_nh.subscribe<rm_msgs::BalanceState>("/state", 1,
                                                                   &WheeledBalanceManual::balanceStateCallback, this);
  x_event_.setRising(boost::bind(&WheeledBalanceManual::xPress, this));
  v_event_.setRising(boost::bind(&WheeledBalanceManual::vPress, this));
  auto_fallen_event_.setActiveHigh(boost::bind(&WheeledBalanceManual::modeFallen, this, _1));
  auto_fallen_event_.setDelayTriggered(boost::bind(&WheeledBalanceManual::modeNormalize, this), 1.5, true);
  ctrl_x_event_.setRising(boost::bind(&WheeledBalanceManual::ctrlXPress, this));
}

void WheeledBalanceManual::sendCommand(const ros::Time& time)
{
  BalanceManual::sendCommand(time);
  balance_chassis_cmd_sender_->sendCommand(time);
}

void WheeledBalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  v_event_.update(dbus_data->key_v && !dbus_data->key_ctrl);
  ctrl_x_event_.update(dbus_data->key_ctrl && dbus_data->key_x);
}

void WheeledBalanceManual::rightSwitchDownRise()
{
  BalanceManual::rightSwitchDownRise();
  balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
}

void WheeledBalanceManual::rightSwitchMidRise()
{
  BalanceManual::rightSwitchMidRise();
  balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
}

void WheeledBalanceManual::ctrlZPress()
{
  BalanceManual::ctrlZPress();
  if (supply_)
  {
    balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  }
  else
  {
    balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  }
}

void WheeledBalanceManual::vPress()
{
  chassis_cmd_sender_->updateSafetyPower(80);
}

void WheeledBalanceManual::bPress()
{
  ChassisGimbalShooterCoverManual::bPress();
  chassis_cmd_sender_->updateSafetyPower(100);
}

void WheeledBalanceManual::ctrlXPress()
{
  if (balance_chassis_cmd_sender_->getMsg()->data == rm_msgs::BalanceState::NORMAL)
    balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  else
    balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
}

void WheeledBalanceManual::balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg)
{
  if ((ros::Time::now() - msg->header.stamp).toSec() < 0.2)
  {
    if (std::abs(msg->theta) > balance_dangerous_angle_)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    if (msg->mode == rm_msgs::BalanceState::NORMAL)
      auto_fallen_event_.update(std::abs(msg->theta) > 0.42 && std::abs(msg->x_dot) > 1.5 &&
                                vel_cmd_sender_->getMsg()->linear.x == 0 && vel_cmd_sender_->getMsg()->linear.y == 0 &&
                                vel_cmd_sender_->getMsg()->angular.z == 0);
  }
}

void WheeledBalanceManual::modeNormalize()
{
  balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  ROS_INFO("mode normalize");
}

void WheeledBalanceManual::modeFallen(ros::Duration duration)
{
  if (duration.toSec() > 0.3)
  {
    balance_chassis_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
    ROS_INFO("mode fallen");
  }
}
}  // namespace rm_manual
