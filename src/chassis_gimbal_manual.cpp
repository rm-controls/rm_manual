//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_manual.h"

namespace rm_manual
{
ChassisGimbalManual::ChassisGimbalManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh);
  ros::NodeHandle vel_nh(nh, "vel");
  vel_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
  if (!vel_nh.getParam("gyro_move_reduction", gyro_move_reduction_))
    ROS_ERROR("Gyro move reduction no defined (namespace: %s)", nh.getNamespace().c_str());
  if (!vel_nh.getParam("gyro_rotate_reduction", gyro_rotate_reduction_))
    ROS_ERROR("Gyro rotate reduction no defined (namespace: %s)", nh.getNamespace().c_str());
  ros::NodeHandle gimbal_nh(nh, "gimbal");
  gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh);

  chassis_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::chassisOutputOn, this));
  gimbal_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::gimbalOutputOn, this));
  w_event_.setEdge(boost::bind(&ChassisGimbalManual::wPress, this), boost::bind(&ChassisGimbalManual::wRelease, this));
  w_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::wPressing, this));
  s_event_.setEdge(boost::bind(&ChassisGimbalManual::sPress, this), boost::bind(&ChassisGimbalManual::sRelease, this));
  s_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::sPressing, this));
  a_event_.setEdge(boost::bind(&ChassisGimbalManual::aPress, this), boost::bind(&ChassisGimbalManual::aRelease, this));
  a_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::aPressing, this));
  d_event_.setEdge(boost::bind(&ChassisGimbalManual::dPress, this), boost::bind(&ChassisGimbalManual::dRelease, this));
  d_event_.setActiveHigh(boost::bind(&ChassisGimbalManual::dPressing, this));
}

void ChassisGimbalManual::sendCommand(const ros::Time& time)
{
  chassis_cmd_sender_->sendCommand(time);
  vel_cmd_sender_->sendCommand(time);
  gimbal_cmd_sender_->sendCommand(time);
}

void ChassisGimbalManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  if (std::abs(dbus_data->wheel) > 0.01)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  }
  else
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setAngularZVel((std::abs(dbus_data->ch_r_y) > 0.01 || std::abs(dbus_data->ch_r_x) > 0.01) ?
                                      dbus_data->wheel * gyro_rotate_reduction_ :
                                      dbus_data->wheel);
  vel_cmd_sender_->setLinearXVel(chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ?
                                     dbus_data->ch_r_y * gyro_move_reduction_ :
                                     dbus_data->ch_r_y);
  vel_cmd_sender_->setLinearYVel(chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ?
                                     -dbus_data->ch_r_x * gyro_move_reduction_ :
                                     -dbus_data->ch_r_x);
  gimbal_cmd_sender_->setRate(-dbus_data->ch_l_x, -dbus_data->ch_l_y);
}

void ChassisGimbalManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  gimbal_cmd_sender_->setRate(-dbus_data->m_x * gimbal_scale_, dbus_data->m_y * gimbal_scale_);
}

void ChassisGimbalManual::checkReferee()
{
  ManualBase::checkReferee();
}

void ChassisGimbalManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::checkKeyboard(dbus_data);
  if (robot_id_ == rm_msgs::GameRobotStatus::RED_ENGINEER || robot_id_ == rm_msgs::GameRobotStatus::BLUE_ENGINEER)
  {
    w_event_.update((!dbus_data->key_ctrl) && dbus_data->key_w);
    s_event_.update((!dbus_data->key_ctrl) && dbus_data->key_s);
    a_event_.update((!dbus_data->key_ctrl) && dbus_data->key_a);
    d_event_.update((!dbus_data->key_ctrl) && dbus_data->key_d);
  }
  else
  {
    w_event_.update(dbus_data->key_w);
    s_event_.update(dbus_data->key_s);
    a_event_.update(dbus_data->key_a);
    d_event_.update(dbus_data->key_d);
  }
  if (dbus_data->m_z != 0)
  {
    mouseMidRise(dbus_data->m_z);
  }
}

void ChassisGimbalManual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  chassis_cmd_sender_->updateGameStatus(*data);
}

void ChassisGimbalManual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  chassis_cmd_sender_->updateGameRobotStatus(*data);
  chassis_power_on_event_.update(data->mains_power_chassis_output);
  gimbal_power_on_event_.update(data->mains_power_gimbal_output);
}

void ChassisGimbalManual::powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
{
  ManualBase::powerHeatDataCallback(data);
  chassis_cmd_sender_->updatePowerHeatData(*data);
}

void ChassisGimbalManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
}

void ChassisGimbalManual::capacityDataCallback(const rm_msgs::CapacityData::ConstPtr& data)
{
  ManualBase::capacityDataCallback(data);
  chassis_cmd_sender_->updateCapacityData(*data);
}

void ChassisGimbalManual::trackCallback(const rm_msgs::TrackData::ConstPtr& data)
{
  ManualBase::trackCallback(data);
}

void ChassisGimbalManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  vel_cmd_sender_->setZero();
  chassis_cmd_sender_->setZero();
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchDownRise()
{
  ManualBase::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::leftSwitchMidFall()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void ChassisGimbalManual::leftSwitchDownRise()
{
  ManualBase::leftSwitchDownRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

double ChassisGimbalManual::judgeMode(double scale)
{
  if (speed_change_mode_)
    scale *= speed_change_scale_;
  if (toward_change_mode_)
    scale *= -1;
  return scale;
}

void ChassisGimbalManual::wPressing()
{
  x_scale_ = judgeMode(x_scale_);
  vel_cmd_sender_->setLinearXVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? x_scale_ * gyro_move_reduction_ : x_scale_);
  vel_cmd_sender_->setAngularZVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalManual::wRelease()
{
  x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  vel_cmd_sender_->setLinearXVel(x_scale_);
  vel_cmd_sender_->setAngularZVel(chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? 1 : 0);
}

void ChassisGimbalManual::sPressing()
{
  x_scale_ = judgeMode(x_scale_);
  vel_cmd_sender_->setLinearXVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? x_scale_ * gyro_move_reduction_ : x_scale_);
  vel_cmd_sender_->setAngularZVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalManual::sRelease()
{
  x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  vel_cmd_sender_->setLinearXVel(x_scale_);
  vel_cmd_sender_->setAngularZVel(chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? 1 : 0);
}

void ChassisGimbalManual::aPressing()
{
  y_scale_ = judgeMode(y_scale_);
  vel_cmd_sender_->setLinearYVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? y_scale_ * gyro_move_reduction_ : y_scale_);
  vel_cmd_sender_->setAngularZVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalManual::aRelease()
{
  y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  vel_cmd_sender_->setLinearYVel(y_scale_);
  vel_cmd_sender_->setAngularZVel(chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? 1 : 0);
}

void ChassisGimbalManual::dPressing()
{
  y_scale_ = judgeMode(y_scale_);
  vel_cmd_sender_->setLinearYVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? y_scale_ * gyro_move_reduction_ : y_scale_);
  vel_cmd_sender_->setAngularZVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalManual::dRelease()
{
  y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  vel_cmd_sender_->setLinearYVel(y_scale_);
  vel_cmd_sender_->setAngularZVel(chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? 1 : 0);
}

void ChassisGimbalManual::mouseMidRise(int m_z)
{
  if (gimbal_scale_ >= 0. && gimbal_scale_ <= 3.)
  {
    if (gimbal_scale_ + 0.2 <= 3. && m_z > 0.)
      gimbal_scale_ += 0.2;
    else if (gimbal_scale_ - 0.2 >= 0. && m_z < 0.)
      gimbal_scale_ -= 0.2;
  }
}

}  // namespace rm_manual
