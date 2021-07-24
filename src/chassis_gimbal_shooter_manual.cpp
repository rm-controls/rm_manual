//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_manual.h"

namespace rm_manual {
ChassisGimbalShooterManual::ChassisGimbalShooterManual(ros::NodeHandle &nh)
    : ChassisGimbalManual(nh),
      q_press_event_(boost::bind(&ChassisGimbalShooterManual::qPress, this, _1)),
      f_press_event_(boost::bind(&ChassisGimbalShooterManual::fPress, this, _1)),
      shift_press_event_(boost::bind(&ChassisGimbalShooterManual::shiftPress, this, _1)),
      shift_release_event_(boost::bind(&ChassisGimbalShooterManual::shiftRelease, this, _1)),
      ctrl_c_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlCPress, this, _1)),
      ctrl_v_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlVPress, this, _1)),
      ctrl_r_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlRPress, this, _1)),
      ctrl_b_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlBPress, this, _1)) {
  ros::NodeHandle shooter_nh(nh, "shooter");
  shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_.referee_.referee_data_);
  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("trigger_calibration", rpc_value);
  trigger_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
}

void ChassisGimbalShooterManual::run() {
  ChassisGimbalManual::run();
  switch_detection_srv_->setEnemyColor(data_.referee_.referee_data_);
  trigger_calibration_->update(ros::Time::now());
}

void ChassisGimbalShooterManual::checkKeyboard() {
  ChassisGimbalManual::checkKeyboard();
  q_press_event_.update(data_.dbus_data_.key_q);
  f_press_event_.update(data_.dbus_data_.key_f);
  shift_press_event_.update(data_.dbus_data_.key_shift);
  shift_release_event_.update(data_.dbus_data_.key_shift);
  ctrl_c_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
  ctrl_v_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v);
  ctrl_r_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  ctrl_b_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_b);
}

void ChassisGimbalShooterManual::sendCommand(const ros::Time &time) {
  ChassisGimbalManual::sendCommand(time);
  shooter_cmd_sender_->sendCommand(time);
}

void ChassisGimbalShooterManual::shooterOutputOn(ros::Duration) {
  ROS_INFO("Shooter Output ON");
  trigger_calibration_->reset();
}

void ChassisGimbalShooterManual::updateRc() {
  ChassisGimbalManual::updateRc();
  if (shooter_cmd_sender_->getMsg()->mode != rm_msgs::ShootCmd::STOP) {
    gimbal_cmd_sender_->updateCost(data_.track_data_array_);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
  }
}

void ChassisGimbalShooterManual::rightSwitchDown(ros::Duration duration) {
  ChassisGimbalManual::rightSwitchDown(duration);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::rightSwitchMid(ros::Duration duration) {
  ChassisGimbalManual::rightSwitchMid(duration);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::rightSwitchUp(ros::Duration duration) {
  ChassisGimbalManual::rightSwitchUp(duration);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::leftSwitchDown(ros::Duration duration) {
  ChassisGimbalManual::leftSwitchDown(duration);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::leftSwitchMid(ros::Duration duration) {
  ChassisGimbalManual::leftSwitchMid(duration);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterManual::leftSwitchUp(ros::Duration duration) {
  ChassisGimbalManual::leftSwitchUp(duration);
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
}

void ChassisGimbalShooterManual::mouseRightPress(ros::Duration) {
  gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  gimbal_cmd_sender_->updateCost(data_.track_data_array_);
  shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
}

void ChassisGimbalShooterManual::xPress(ros::Duration duration) {
  ChassisGimbalManual::xPress(duration);
  fixed_ui_->add();
}

void ChassisGimbalShooterManual::ctrlVPress(ros::Duration) {
  switch_detection_srv_->switchEnemyColor();
  switch_detection_srv_->callService();
}

void ChassisGimbalShooterManual::ctrlRPress(ros::Duration) {
  switch_detection_srv_->switchTargetType();
  switch_detection_srv_->callService();
  if (switch_detection_srv_->getTarget())
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  else
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
}

void ChassisGimbalShooterManual::ctrlBPress(ros::Duration) {
  switch_detection_srv_->switchExposureLevel();
  switch_detection_srv_->callService();
}

void ChassisGimbalShooterManual::drawUi(const ros::Time &time) {
  ChassisGimbalManual::drawUi(time);
  trigger_change_ui_->update("target", switch_detection_srv_->getTarget(), shooter_cmd_sender_->getBurstMode(),
                             switch_detection_srv_->getColor() == rm_msgs::StatusChangeRequest::RED);
  fixed_ui_->update();
}
}
