//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"

namespace rm_manual {
ChassisGimbalShooterCoverManual::ChassisGimbalShooterCoverManual(ros::NodeHandle &nh) : ChassisGimbalShooterManual(nh) {
  ros::NodeHandle cover_nh(nh, "cover");
  nh.param("supply_frame", supply_frame_);
  cover_command_sender_ =
      new rm_common::JointPositionBinaryCommandSender(cover_nh);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("gimbal_calibration", rpc_value);
  gimbal_calibration_ =
      new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  ctrl_z_event_.setEdge(
      boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this),
      boost::bind(&ChassisGimbalShooterCoverManual::ctrlZRelease, this));
  ctrl_q_event_.setRising(
      boost::bind(&ChassisGimbalShooterCoverManual::ctrlQPress, this));
}

void ChassisGimbalShooterCoverManual::run() {
  ChassisGimbalShooterManual::run();
  gimbal_calibration_->update(ros::Time::now());
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
  if (supply_) {
    chassis_cmd_sender_->getMsg()->follow_source_frame = supply_frame_;
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(
        rm_common::PowerLimit::NORMAL);
    double roll, pitch, yaw;
    quatToRPY(data_.tf_buffer_
                  .lookupTransform("base_link", supply_frame_, ros::Time(0))
                  .transform.rotation,
              roll, pitch, yaw);
    if (std::abs(yaw) < 0.05)
      cover_command_sender_->on();
  } else {
    cover_command_sender_->off();
    double roll, pitch, yaw;
    quatToRPY(
        data_.tf_buffer_.lookupTransform("base_link", "cover", ros::Time(0))
            .transform.rotation,
        roll, pitch, yaw);
    if (std::abs(pitch) > 0.05) {
      chassis_cmd_sender_->getMsg()->follow_source_frame = supply_frame_;
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
      chassis_cmd_sender_->power_limit_->updateState(
          rm_common::PowerLimit::NORMAL);
    }
  }
  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
}

void ChassisGimbalShooterCoverManual::gimbalOutputOn() {
  ChassisGimbalShooterManual::gimbalOutputOn();
  gimbal_calibration_->reset();
}

void ChassisGimbalShooterCoverManual::remoteControlTurnOff() {
  ChassisGimbalShooterManual::remoteControlTurnOff();
  gimbal_calibration_->stop();
}

void ChassisGimbalShooterCoverManual::remoteControlTurnOn() {
  ChassisGimbalShooterManual::remoteControlTurnOn();
  gimbal_calibration_->stopController();
}

void ChassisGimbalShooterCoverManual::drawUi(const ros::Time &time) {
  ChassisGimbalShooterManual::drawUi(time);
  flash_ui_->update("cover", time, !cover_command_sender_->getState());
}

void ChassisGimbalShooterCoverManual::rightSwitchDownRise() {
  ChassisGimbalShooterManual::rightSwitchDownRise();
  supply_ = true;
}

void ChassisGimbalShooterCoverManual::rightSwitchMidRise() {
  ChassisGimbalShooterManual::rightSwitchMidRise();
  supply_ = false;
}

void ChassisGimbalShooterCoverManual::rightSwitchUpRise() {
  ChassisGimbalShooterManual::rightSwitchUpRise();
  supply_ = false;
}

void ChassisGimbalShooterCoverManual::ctrlZPress() {
  supply_ = !cover_command_sender_->getState();
}

void ChassisGimbalShooterCoverManual::ctrlQPress() {
  gimbal_calibration_->reset();
}
}
