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
  z_event_.setRising(boost::bind(&BalanceManual::zPress, this));
  x_event_.setRising(boost::bind(&BalanceManual::xPress, this));
  r_event_.setRising(boost::bind(&BalanceManual::rPress, this));
  g_event_.setRising(boost::bind(&BalanceManual::gPress, this));
  auto_fallen_event_.setActiveHigh(boost::bind(&BalanceManual::modeFallen, this, _1));
  auto_fallen_event_.setDelayTriggered(boost::bind(&BalanceManual::modeNormalize, this), 1.5, true);
  ctrl_x_event_.setRising(boost::bind(&BalanceManual::ctrlXPress, this));

  ramp_x_ = new RampFilter<double>(0, 0.01);
  ramp_x_->setAcc(10);
  ramp_y_ = new RampFilter<double>(0, 0.01);
  ramp_y_->setAcc(10);

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
  z_event_.update(dbus_data->key_z && !dbus_data->key_ctrl);
  x_event_.update(dbus_data->key_x && !dbus_data->key_ctrl);
  r_event_.update(dbus_data->key_r && !dbus_data->key_ctrl);
  g_event_.update(dbus_data->key_g && !dbus_data->key_ctrl);
  ctrl_x_event_.update(dbus_data->key_ctrl && dbus_data->key_x);
}

void BalanceManual::shiftRelease()
{
}

void BalanceManual::shiftPress()
{
  ChassisGimbalShooterCoverManual::shiftPress();
  chassis_cmd_sender_->updateSafetyPower(60);
}

void BalanceManual::zPress()
{
  chassis_cmd_sender_->updateSafetyPower(80);
}

void BalanceManual::xPress()
{
  chassis_cmd_sender_->updateSafetyPower(100);
}

void BalanceManual::rPress()
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
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
    is_gyro_ = false;
  }
  else
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    gyro_scale_ = 0.0;
    is_gyro_ = true;
    gyro_timer_.start();
  }
}

void BalanceManual::wPress()
{
  ramp_x_->clear();
}

void BalanceManual::wPressing()
{
  double final_x_scale;
  ramp_x_->input(1.0);
  final_x_scale = ramp_x_->output();
  vel_cmd_sender_->setLinearXVel(final_x_scale > 1.0 ? 1.0 : final_x_scale);
}

void BalanceManual::sPress()
{
  ramp_x_->clear();
}

void BalanceManual::sPressing()
{
  double final_x_scale;
  ramp_x_->input(1.0);
  final_x_scale = -ramp_x_->output();
  vel_cmd_sender_->setLinearXVel(final_x_scale < -1.0 ? -1.0 : final_x_scale);
}

void BalanceManual::aPress()
{
  ramp_y_->clear();
}

void BalanceManual::aPressing()
{
  double final_y_scale;
  ramp_y_->input(1.0);
  final_y_scale = ramp_y_->output();
  vel_cmd_sender_->setLinearYVel(final_y_scale > 1.0 ? 1.0 : final_y_scale);
}

void BalanceManual::dPress()
{
  ramp_y_->clear();
}

void BalanceManual::dPressing()
{
  double final_y_scale;
  ramp_y_->input(1.0);
  final_y_scale = -ramp_y_->output();
  vel_cmd_sender_->setLinearYVel(final_y_scale < -1.0 ? -1.0 : final_y_scale);
}

void BalanceManual::wRelease()
{
  vel_cmd_sender_->setLinearXVel(0);
}

void BalanceManual::sRelease()
{
  vel_cmd_sender_->setLinearXVel(0);
}

void BalanceManual::aRelease()
{
  vel_cmd_sender_->setLinearYVel(0);
}

void BalanceManual::dRelease()
{
  vel_cmd_sender_->setLinearYVel(0);
}

void BalanceManual::ctrlXPress()
{
  if (balance_cmd_sender_->getMsg()->data == rm_msgs::BalanceState::NORMAL)
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  else
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
}

void BalanceManual::balanceStateCallback(const rm_msgs::BalanceState::ConstPtr& msg)
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

void BalanceManual::gyroCPressedCallback()
{
  if ((gyro_scale_ >= 1.0 && is_gyro_) || (gyro_scale_ <= 0 && !is_gyro_))
  {
    gyro_timer_.stop();
    return;
  }

  if (is_gyro_)
    gyro_scale_ = gyro_scale_ + 10 * 0.01 > 1.0 ? 1.0 : gyro_scale_ + 10 * 0.01;
  else
    gyro_scale_ = gyro_scale_ - 10 * 0.01 < 0 ? 0 : gyro_scale_ - 10 * 0.01;
  vel_cmd_sender_->setAngularZVel(gyro_scale_);
}

void BalanceManual::modeNormalize()
{
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
  ROS_INFO("mode normalize");
}

void BalanceManual::modeFallen(ros::Duration duration)
{
  if (duration.toSec() > 0.3)
  {
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
    ROS_INFO("mode fallen");
  }
}
}  // namespace rm_manual
