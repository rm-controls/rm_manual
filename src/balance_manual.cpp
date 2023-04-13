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

  nh.param("flank_frame", flank_frame_, std::string("flank_frame"));
  nh.param("reverse_frame", reverse_frame_, std::string("yaw_reverse_frame"));
  nh.param("balance_dangerous_angle", balance_dangerous_angle_, 0.3);

  is_balance_ = true;
  state_sub_ = balance_nh.subscribe<rm_msgs::BalanceState>("/state", 1, &BalanceManual::balanceStateCallback, this);
  v_event_.setRising(boost::bind(&BalanceManual::vPress, this));
  g_event_.setRising(boost::bind(&BalanceManual::gPress, this));
  ctrl_x_event_.setDelayTriggered(boost::bind(&BalanceManual::modeNormalizeDelay, this), 3.0, true);
  ctrl_x_event_.setRising(boost::bind(&BalanceManual::ctrlXPress, this));

  gyro_timer_ =
      nh.createTimer(ros::Duration(0.03), std::bind(&BalanceManual::gyroCPressedCallback, this), false, false);
}

void BalanceManual::sendCommand(const ros::Time& time)
{
  if (flank_)
    chassis_cmd_sender_->getMsg()->follow_source_frame = flank_frame_;
  else if (reverse_)
    chassis_cmd_sender_->getMsg()->follow_source_frame = reverse_frame_;
  else
    chassis_cmd_sender_->getMsg()->follow_source_frame = "yaw";

  ChassisGimbalShooterCoverManual::sendCommand(time);
  balance_cmd_sender_->sendCommand(time);
}

void BalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  v_event_.update(dbus_data->key_v);
  g_event_.update(dbus_data->key_g);
  ctrl_x_event_.update(dbus_data->key_ctrl && dbus_data->key_x);
}

void BalanceManual::vPress()
{
  flank_ = !flank_;
  if (reverse_)
    reverse_ = false;
}

void BalanceManual::gPress()
{
  reverse_ = !reverse_;
  if (flank_)
    flank_ = false;
}

void BalanceManual::cPress()
{
  if (is_gyro_)
    gyro_scale_ = 1.0;
  else
    gyro_scale_ = 0.0;
  ChassisGimbalShooterCoverManual::cPress();
  gyro_timer_.start();
}

void BalanceManual::wPress()
{
  ChassisGimbalShooterCoverManual::wPress();
}

void BalanceManual::sPress()
{
  ChassisGimbalShooterCoverManual::sPress();
}

void BalanceManual::aPress()
{
  ChassisGimbalShooterCoverManual::aPress();
}

void BalanceManual::dPress()
{
  ChassisGimbalShooterCoverManual::dPress();
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
    if (std::abs(msg->theta) > balance_dangerous_angle_)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    else
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void BalanceManual::gyroCPressedCallback()
{
  if ((gyro_scale_ >= 1.0 && is_gyro_) || (gyro_scale_ <= 0 && !is_gyro_))
  {
    gyro_timer_.stop();
    return;
  }

  if (is_gyro_)
    gyro_scale_ = gyro_scale_ + 8 * 0.01 > 1.0 ? 1.0 : gyro_scale_ + 8 * 0.01;
  else
    gyro_scale_ = gyro_scale_ - 8 * 0.01 < 0 ? 0 : gyro_scale_ - 8 * 0.01;
  vel_cmd_sender_->setAngularZVel(gyro_scale_);
}

void BalanceManual::modeNormalizeDelay()
{
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  ROS_INFO("mode normalize");
}
}  // namespace rm_manual
