//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"

namespace rm_manual {
ChassisGimbalShooterCoverManual::ChassisGimbalShooterCoverManual(ros::NodeHandle &nh) : ChassisGimbalShooterManual(nh) {
  ros::NodeHandle cover_nh(nh, "cover");
  cover_command_sender_ = new rm_common::JointPositionBinaryCommandSender(cover_nh);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("cover_calibration", rpc_value);
  cover_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  ctrl_z_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this),
                        boost::bind(&ChassisGimbalShooterCoverManual::ctrlZRelease, this));
  ctrl_q_event_.setRising(boost::bind(&ChassisGimbalShooterCoverManual::ctrlQPress, this));
}

void ChassisGimbalShooterCoverManual::run() {
  ChassisGimbalShooterManual::run();
  cover_calibration_->update(ros::Time::now());
}

void ChassisGimbalShooterCoverManual::updatePc() {
  ChassisGimbalShooterManual::updatePc();
  gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x * gimbal_scale_,
                              cover_command_sender_->getState() ? 0.0 : data_.dbus_data_.m_y * gimbal_scale_);
}

void ChassisGimbalShooterCoverManual::checkKeyboard() {
  ChassisGimbalShooterManual::checkKeyboard();
  ctrl_z_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z);
  ctrl_q_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_q);
}

void ChassisGimbalShooterCoverManual::sendCommand(const ros::Time &time) {
  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
}

void ChassisGimbalShooterCoverManual::gimbalOutputOn() {
  ChassisGimbalShooterManual::gimbalOutputOn();
  cover_calibration_->reset();
}

void ChassisGimbalShooterCoverManual::remoteControlTurnOff() {
  ChassisGimbalShooterManual::remoteControlTurnOff();
  cover_calibration_->stop();
}

void ChassisGimbalShooterCoverManual::remoteControlTurnOn() {
  ChassisGimbalShooterManual::remoteControlTurnOn();
  cover_calibration_->stopController();
}

void ChassisGimbalShooterCoverManual::rightSwitchDownRise() {
  ChassisGimbalShooterManual::rightSwitchDownRise();
  cover_command_sender_->on();
}

void ChassisGimbalShooterCoverManual::rightSwitchMidRise() {
  ChassisGimbalShooterManual::rightSwitchMidRise();
  cover_command_sender_->off();
}

void ChassisGimbalShooterCoverManual::rightSwitchUpRise() {
  ChassisGimbalShooterManual::rightSwitchUpRise();
  cover_command_sender_->off();
}

void ChassisGimbalShooterCoverManual::ctrlZPress() {
  geometry_msgs::PointStamped aim_point{};
  aim_point.header.frame_id = "yaw";
  aim_point.header.stamp = ros::Time(0);
  aim_point.point.x = 1;
  aim_point.point.y = 0;
  aim_point.point.z = 0;
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  cover_command_sender_->getState() ? cover_command_sender_->off() : cover_command_sender_->on();
}

void ChassisGimbalShooterCoverManual::ctrlQPress() {
  cover_calibration_->reset();
}
}
