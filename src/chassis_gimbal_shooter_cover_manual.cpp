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
  ctrl_z_event_.setRising(boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this));
}

void ChassisGimbalShooterCoverManual::run() {
  ChassisGimbalShooterManual::run();
  cover_calibration_->update(ros::Time::now());
}

void ChassisGimbalShooterCoverManual::checkKeyboard() {
  ChassisGimbalShooterManual::checkKeyboard();
  ctrl_z_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z);
}

void ChassisGimbalShooterCoverManual::sendCommand(const ros::Time &time) {
  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
}

void ChassisGimbalShooterCoverManual::shooterOutputOn() {
  ChassisGimbalShooterManual::shooterOutputOn();
  cover_calibration_->reset();
}

void ChassisGimbalShooterCoverManual::drawUi(const ros::Time &time) {
  ChassisGimbalShooterManual::drawUi(time);
  flash_ui_->update("cover", time, cover_command_sender_->getState());
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

void ChassisGimbalShooterCoverManual::mouseRightPress() {
  if (cover_command_sender_->getState())
    ChassisGimbalShooterManual::mouseRightPress();
}

void ChassisGimbalShooterCoverManual::mouseRightRelease() {
  if (cover_command_sender_->getState())
    ChassisGimbalShooterManual::mouseRightRelease();
}

void ChassisGimbalShooterCoverManual::ctrlZPress() {
  if (cover_command_sender_->getState()) {
    geometry_msgs::PointStamped aim_point{};
    aim_point.header.frame_id = "yaw";
    aim_point.header.stamp = ros::Time(0);
    aim_point.point.x = 1000;
    aim_point.point.y = 0;
    aim_point.point.z = 0;
    gimbal_cmd_sender_->setAimPoint(aim_point);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
    cover_command_sender_->on();
  } else {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    cover_command_sender_->off();
  }
}
}
