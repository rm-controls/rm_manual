//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_manual.h"

namespace rm_manual {
ChassisGimbalManual::ChassisGimbalManual(ros::NodeHandle &nh) : ManualBase(nh) {
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, data_.referee_.referee_data_);
  ros::NodeHandle vel_nh(nh, "vel");
  vel_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
  if (!vel_nh.getParam("gyro_move_reduction", gyro_move_reduction_))
    ROS_ERROR("Gyro move reduction no defined (namespace: %s)", nh.getNamespace().c_str());
  ros::NodeHandle gimbal_nh(nh, "gimbal");
  gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_.referee_.referee_data_);
  ros::NodeHandle ui_nh(nh, "ui");
  time_change_ui_ = new TimeChangeUi(ui_nh, data_);
  flash_ui_ = new FlashUi(ui_nh, data_);
  trigger_change_ui_ = new TriggerChangeUi(ui_nh, data_);
  fixed_ui_ = new FixedUi(ui_nh, data_);

  chassis_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::chassisOutputOn, this));
  gimbal_power_on_event_.setRising(boost::bind(&ChassisGimbalManual::gimbalOutputOn, this));
  w_event_.setEdge(boost::bind(&ChassisGimbalManual::wPress, this),
                   boost::bind(&ChassisGimbalManual::wRelease, this));
  s_event_.setEdge(boost::bind(&ChassisGimbalManual::sPress, this),
                   boost::bind(&ChassisGimbalManual::sRelease, this));
  a_event_.setEdge(boost::bind(&ChassisGimbalManual::aPress, this),
                   boost::bind(&ChassisGimbalManual::aRelease, this));
  d_event_.setEdge(boost::bind(&ChassisGimbalManual::dPress, this),
                   boost::bind(&ChassisGimbalManual::dRelease, this));
  mouse_left_event_.setEdge(boost::bind(&ChassisGimbalManual::mouseLeftPress, this),
                            boost::bind(&ChassisGimbalManual::mouseLeftRelease, this));
  mouse_right_event_.setEdge(boost::bind(&ChassisGimbalManual::mouseRightPress, this),
                             boost::bind(&ChassisGimbalManual::mouseRightRelease, this));
}

void ChassisGimbalManual::sendCommand(const ros::Time &time) {
  chassis_cmd_sender_->sendCommand(time);
  vel_cmd_sender_->sendCommand(time);
  gimbal_cmd_sender_->sendCommand(time);
}

void ChassisGimbalManual::updateRc() {
  ManualBase::updateRc();
  if (std::abs(data_.dbus_data_.wheel) > 0.01) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  } else
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setAngularZVel(data_.dbus_data_.wheel);
  vel_cmd_sender_->setLinearXVel(data_.dbus_data_.ch_r_y);
  vel_cmd_sender_->setLinearYVel(-data_.dbus_data_.ch_r_x);
  gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
}

void ChassisGimbalManual::updatePc() {
  ManualBase::updatePc();
  gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x, data_.dbus_data_.m_y);
}

void ChassisGimbalManual::checkReferee() {
  chassis_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_);
  gimbal_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_);
}

void ChassisGimbalManual::checkKeyboard() {
  w_event_.update(data_.dbus_data_.key_w);
  s_event_.update(data_.dbus_data_.key_s);
  a_event_.update(data_.dbus_data_.key_a);
  d_event_.update(data_.dbus_data_.key_d);
  mouse_left_event_.update(data_.dbus_data_.p_l);
  mouse_right_event_.update(data_.dbus_data_.p_r);
}

void ChassisGimbalManual::drawUi(const ros::Time &time) {
  ManualBase::drawUi(time);
  time_change_ui_->update("capacitor", time);
  flash_ui_->update("spin", time,
                    chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO
                        && vel_cmd_sender_->getMsg()->angular.z != 0.);
  trigger_change_ui_->update("chassis", chassis_cmd_sender_->getMsg()->mode,
                             chassis_cmd_sender_->getBurstMode(), chassis_cmd_sender_->getChargeMode());
  flash_ui_->update("armor0", time);
  flash_ui_->update("armor1", time);
  flash_ui_->update("armor2", time);
  flash_ui_->update("armor3", time);
}

void ChassisGimbalManual::remoteControlTurnOff() {
  ManualBase::remoteControlTurnOff();
  vel_cmd_sender_->setZero();
  chassis_cmd_sender_->setZero();
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchDownRise() {
  ManualBase::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  gimbal_cmd_sender_->setZero();
}

void ChassisGimbalManual::rightSwitchMidRise() {
  ManualBase::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::rightSwitchUpRise() {
  ManualBase::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  vel_cmd_sender_->setZero();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  trigger_change_ui_->add();
  time_change_ui_->add();
  fixed_ui_->add();
}

void ChassisGimbalManual::leftSwitchDownRise() {
  ManualBase::leftSwitchDownRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalManual::wPress() {
  x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  vel_cmd_sender_->setLinearXVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? x_scale_ * gyro_move_reduction_ : x_scale_);
}
void ChassisGimbalManual::wRelease() {
  x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  vel_cmd_sender_->setLinearXVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? x_scale_ * gyro_move_reduction_ : x_scale_);
}
void ChassisGimbalManual::aPress() {
  y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  vel_cmd_sender_->setLinearYVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? y_scale_ * gyro_move_reduction_ : y_scale_);
}
void ChassisGimbalManual::aRelease() {
  y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  vel_cmd_sender_->setLinearYVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? y_scale_ * gyro_move_reduction_ : y_scale_);
}
void ChassisGimbalManual::sPress() {
  x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  vel_cmd_sender_->setLinearXVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? x_scale_ * gyro_move_reduction_ : x_scale_);
}
void ChassisGimbalManual::sRelease() {
  x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  vel_cmd_sender_->setLinearXVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? x_scale_ * gyro_move_reduction_ : x_scale_);
}
void ChassisGimbalManual::dPress() {
  y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  vel_cmd_sender_->setLinearYVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? y_scale_ * gyro_move_reduction_ : y_scale_);
}
void ChassisGimbalManual::dRelease() {
  y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  vel_cmd_sender_->setLinearYVel(
      chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO ? y_scale_ * gyro_move_reduction_ : y_scale_);
}

}
