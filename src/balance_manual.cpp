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

  ros::NodeHandle left_block_nh(nh, "left_momentum_block");
  left_momentum_block_cmd_sender_ = new rm_common::JointPointCommandSender(left_block_nh, joint_state_);
  ros::NodeHandle right_block_nh(nh, "right_momentum_block");
  right_momentum_block_cmd_sender_ = new rm_common::JointPointCommandSender(right_block_nh, joint_state_);

  nh.param("flank_frame", flank_frame_, std::string("flank_frame"));
  nh.param("reverse_frame", reverse_frame_, std::string("yaw_reverse_frame"));
  nh.param("balance_dangerous_angle", balance_dangerous_angle_, 0.3);

  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("shooter_calibration", rpc_value);

  XmlRpc::XmlRpcValue controllers;
  if (nh.getParam("lqr_controllers", controllers))
    for (int i = 0; i < controllers.size(); ++i)
    {
      lqr_controllers_.push_back(controllers[i]);
      controller_manager_.loadController(lqr_controllers_.back());
    }
  else
    ROS_ERROR("can't get lqr_controllers in ns: %s", nh.getNamespace().c_str());
  if (nh.getParam("mpc_controllers", controllers))
    for (int i = 0; i < controllers.size(); ++i)
    {
      mpc_controllers_.push_back(controllers[i]);
      controller_manager_.loadController(mpc_controllers_.back());
    }
  else
    ROS_ERROR("can't get mpc_controllers in ns: %s", nh.getNamespace().c_str());

  is_balance_ = true;
  state_sub_ = balance_nh.subscribe<rm_msgs::BalanceState>("/state", 1, &BalanceManual::balanceStateCallback, this);
  x_event_.setRising(boost::bind(&BalanceManual::xPress, this));
  g_event_.setRising(boost::bind(&BalanceManual::gPress, this));
  v_event_.setRising(boost::bind(&BalanceManual::vPress, this));
  auto_fallen_event_.setActiveHigh(boost::bind(&BalanceManual::modeFallen, this, _1));
  auto_fallen_event_.setDelayTriggered(boost::bind(&BalanceManual::modeNormalize, this), 1.5, true);
  ctrl_x_event_.setRising(boost::bind(&BalanceManual::ctrlXPress, this));
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

  if (control_method_change_)
  {
    switch (control_method_)
    {
      case rm_msgs::ManualToReferee::LQR:
        controller_manager_.startControllers(lqr_controllers_);
        controller_manager_.stopControllers(mpc_controllers_);
        break;
      case rm_msgs::ManualToReferee::MPC:
        controller_manager_.startControllers(mpc_controllers_);
        controller_manager_.stopControllers(lqr_controllers_);
        break;
    }
    control_method_change_ = false;
  }
  cover_command_sender_->sendCommand(time);
  balance_cmd_sender_->sendCommand(time);

  if (control_method_ == rm_msgs::ManualToReferee::MPC)
  {
    left_momentum_block_cmd_sender_->getMsg()->data = 0;
    right_momentum_block_cmd_sender_->getMsg()->data = 0;
    left_momentum_block_cmd_sender_->sendCommand(time);
    right_momentum_block_cmd_sender_->sendCommand(time);
  }
  ChassisGimbalShooterManual::sendCommand(time);
}

void BalanceManual::remoteControlTurnOn()
{
  ChassisGimbalShooterCoverManual::remoteControlTurnOn();
  switch (control_method_)
  {
    case rm_msgs::ManualToReferee::LQR:
      controller_manager_.startControllers(lqr_controllers_);
      controller_manager_.stopControllers(mpc_controllers_);
      break;
    case rm_msgs::ManualToReferee::MPC:
      controller_manager_.startControllers(mpc_controllers_);
      controller_manager_.stopControllers(lqr_controllers_);
      break;
  }
}

void BalanceManual::remoteControlTurnOff()
{
  ChassisGimbalShooterCoverManual::remoteControlTurnOff();
  controller_manager_.stopControllers(mpc_controllers_);
  controller_manager_.stopControllers(lqr_controllers_);
}

void BalanceManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  v_event_.update(dbus_data->key_v && !dbus_data->key_ctrl);
  ctrl_x_event_.update(dbus_data->key_ctrl && dbus_data->key_x);
}

void BalanceManual::checkReferee()
{
  manual_to_referee_pub_data_.balance_control_method = control_method_;
  ChassisGimbalShooterCoverManual::checkReferee();
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
}

void BalanceManual::rightSwitchMidRise()
{
  ChassisGimbalShooterCoverManual::rightSwitchMidRise();
  balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
}

void BalanceManual::ctrlZPress()
{
  ChassisGimbalShooterCoverManual::ctrlZPress();
  if (supply_)
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::FALLEN);
  else
    balance_cmd_sender_->setBalanceMode(rm_msgs::BalanceState::NORMAL);
}

void BalanceManual::shiftRelease()
{
}

void BalanceManual::shiftPress()
{
  if (control_method_ == rm_msgs::ManualToReferee::LQR)
    control_method_ = rm_msgs::ManualToReferee::MPC;
  else if (control_method_ == rm_msgs::ManualToReferee::MPC)
    control_method_ = rm_msgs::ManualToReferee::LQR;
  control_method_change_ = true;
  ChassisGimbalShooterCoverManual::shiftPress();
  chassis_cmd_sender_->updateSafetyPower(60);
}

void BalanceManual::vPress()
{
  chassis_cmd_sender_->updateSafetyPower(80);
}

void BalanceManual::bPress()
{
  ChassisGimbalShooterCoverManual::bPress();
  chassis_cmd_sender_->updateSafetyPower(100);
}

void BalanceManual::wPress()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::wPress();
}

void BalanceManual::wPressing()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::wPressing();
}

void BalanceManual::sPress()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::sPress();
}

void BalanceManual::sPressing()
{
  if (flank_)
    flank_ = !flank_;
  ChassisGimbalShooterCoverManual::sPressing();
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
