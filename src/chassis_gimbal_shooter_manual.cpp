//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_manual.h"

namespace rm_manual {
ChassisGimbalShooterManual::ChassisGimbalShooterManual(ros::NodeHandle &nh) : ChassisGimbalManual(nh) {
  ros::NodeHandle shooter_nh(nh, "shooter");
  shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_.referee_.referee_data_);
  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);
  ros::NodeHandle ui_nh(nh, "ui");
  aim_ui_ = new AimUi(ui_nh, data_.referee_);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("trigger_calibration", rpc_value);
  trigger_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  e_event_.setRising([this] { ePress(); });
  g_event_.setRising([this] { gPress(); });
  q_event_.setRising([this] { qPress(); });
  f_event_.setRising([this] { fPress(); });
  ctrl_c_event_.setRising([this] { ctrlCPress(); });
  ctrl_v_event_.setRising([this] { ctrlVPress(); });
  ctrl_r_event_.setRising([this] { ctrlRPress(); });
  ctrl_b_event_.setRising([this] { ctrlBPress(); });
  shift_event_.setEdge([this] { shiftPress(); }, [this] { shiftRelease(); });
}

void ChassisGimbalShooterManual::run() {
  ChassisGimbalManual::run();
  switch_detection_srv_->setEnemyColor(data_.referee_.referee_data_);
  trigger_calibration_->update(ros::Time::now());
}

void ChassisGimbalShooterManual::checkKeyboard() {
  ChassisGimbalManual::checkKeyboard();
  e_event_.update(data_.dbus_data_.key_e);
  g_event_.update(data_.dbus_data_.key_g);
  q_event_.update(data_.dbus_data_.key_q);
  f_event_.update(data_.dbus_data_.key_f);
  ctrl_c_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
  ctrl_v_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v);
  ctrl_r_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  ctrl_b_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_b);
  shift_event_.update(data_.dbus_data_.key_shift);
}

void ChassisGimbalShooterManual::sendCommand(const ros::Time &time) {
  ChassisGimbalManual::sendCommand(time);
  shooter_cmd_sender_->sendCommand(time);
}

void ChassisGimbalShooterManual::shooterOutputOn() {
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

void ChassisGimbalShooterManual::updatePc() {
  ChassisGimbalManual::updatePc();
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO) {
    if (vel_cmd_sender_->getMsg()->linear.x != 0 || vel_cmd_sender_->getMsg()->linear.y != 0)
      vel_cmd_sender_->setAngularZVel(gyro_move_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(1.0);
  }
}

void ChassisGimbalShooterManual::rightSwitchDown() {
  ChassisGimbalManual::rightSwitchDown();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::rightSwitchMid() {
  ChassisGimbalManual::rightSwitchMid();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::rightSwitchUp() {
  ChassisGimbalManual::rightSwitchUp();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::leftSwitchDown() {
  ChassisGimbalManual::leftSwitchDown();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::leftSwitchMid() {
  ChassisGimbalManual::leftSwitchMid();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterManual::leftSwitchUp() {
  ChassisGimbalManual::leftSwitchUp();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
}

void ChassisGimbalShooterManual::mouseRightPress() {
  gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  gimbal_cmd_sender_->updateCost(data_.track_data_array_);
  shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
}

void ChassisGimbalShooterManual::gPress() {
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO) {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
  } else {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    vel_cmd_sender_->setAngularZVel(1.0);
  }
}

void ChassisGimbalShooterManual::ePress() {
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::TWIST)
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  else
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::TWIST);
}

void ChassisGimbalShooterManual::xPress() {
  ChassisGimbalManual::xPress();
  aim_ui_->add();
}

void ChassisGimbalShooterManual::ctrlVPress() {
  switch_detection_srv_->switchEnemyColor();
  switch_detection_srv_->callService();
}

void ChassisGimbalShooterManual::ctrlRPress() {
  switch_detection_srv_->switchTargetType();
  switch_detection_srv_->callService();
  if (switch_detection_srv_->getTarget())
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  else
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
}

void ChassisGimbalShooterManual::ctrlBPress() {
  switch_detection_srv_->switchExposureLevel();
  switch_detection_srv_->callService();
}

void ChassisGimbalShooterManual::drawUi() {
  ChassisGimbalManual::drawUi();
  state_ui_->update("shooter", shooter_cmd_sender_->getMsg()->mode, shooter_cmd_sender_->getBurstMode());
  state_ui_->update("target", switch_detection_srv_->getTarget(),
                    switch_detection_srv_->getColor() == rm_msgs::StatusChangeRequest::RED);
  aim_ui_->update();
}
}
