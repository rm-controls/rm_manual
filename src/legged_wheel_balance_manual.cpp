//
// Created by kook on 9/28/24.
//

#include "rm_manual/legged_wheel_balance_manual.h"

namespace rm_manual
{
LeggedWheelBalanceManual::LeggedWheelBalanceManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : BalanceManual(nh, nh_referee)
{
  ros::NodeHandle leg_wheel_chassis_nh(nh, "balance/legged_wheel_chassis");
  legCommandSender_ = new rm_common::LegCommandSender(leg_wheel_chassis_nh);
  legCommandSender_->setLgeLength(0.18);
  legCommandSender_->setJump(false);

  b_event_.setEdge(boost::bind(&LeggedWheelBalanceManual::bPress, this),
                   boost::bind(&LeggedWheelBalanceManual::bRelease, this));
  r_event_.setEdge(boost::bind(&LeggedWheelBalanceManual::rPress, this),
                   boost::bind(&LeggedWheelBalanceManual::rRelease, this));
  ctrl_g_event_.setRising(boost::bind(&LeggedWheelBalanceManual::ctrlGPress, this));

  std::string unstick_topic;
  leg_wheel_chassis_nh.param("unstick_topic", unstick_topic,
                             std::string("/controllers/legged_balance_controller/unstick/two_leg_unstick"));
  unstick_sub_ = leg_wheel_chassis_nh.subscribe<std_msgs::Bool>(unstick_topic, 1,
                                                                &LeggedWheelBalanceManual::unstickCallback, this);
}

void LeggedWheelBalanceManual::sendCommand(const ros::Time& time)
{
  BalanceManual::sendCommand(time);
  if (is_gyro_)
  {
    double current_length = legCommandSender_->getLgeLength();
    if (is_increasing_length_)
    {
      if (current_length < 0.3)
      {
        double delta = current_length + 0.002;
        legCommandSender_->setLgeLength(delta > 0.3 ? 0.3 : delta);
      }
      else
        is_increasing_length_ = false;
    }
    else
    {
      if (current_length > 0.18)
      {
        double delta = current_length - 0.002;
        legCommandSender_->setLgeLength(delta < 0.18 ? 0.18 : delta);
      }
      else
        is_increasing_length_ = true;
    }
  }
  legCommandSender_->sendCommand(time);
}

void LeggedWheelBalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  ctrl_g_event_.update(dbus_data->key_ctrl && dbus_data->key_g);
  ctrl_event_.update(dbus_data->key_ctrl);
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
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
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
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
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

void LeggedWheelBalanceManual::rPress()
{
  legCommandSender_->setJump(true);
}

void LeggedWheelBalanceManual::rRelease()
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
    legCommandSender_->setLgeLength(0.3);
    stretch_ = true;
  }
  else
  {
    legCommandSender_->setLgeLength(0.18);
    stretch_ = false;
  }
}

void LeggedWheelBalanceManual::unstickCallback(const std_msgs::BoolConstPtr& msg)
{
  auto two_leg_unstick = msg->data;
  if (two_leg_unstick)
  {
    auto delta = legCommandSender_->getLgeLength() + 0.02;
    legCommandSender_->setLgeLength(delta > 0.33 ? 0.33 : delta);
    stretching_ = true;
  }
  else if (stretching_ && !two_leg_unstick)
  {
    legCommandSender_->setLgeLength(0.18);
    stretching_ = false;
  }
}

}  // namespace rm_manual
