//
// Created by chenzheng on 7/20/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_COVER_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_COVER_MANUAL_H_
#include "rm_manual/chassis_gimbal_shooter_manual.h"

namespace rm_manual {
class ChassisGimbalShooterCoverManual : public ChassisGimbalShooterManual {
 public:
  explicit ChassisGimbalShooterCoverManual(ros::NodeHandle &nh)
      : ChassisGimbalShooterManual(nh),
        ctrl_z_press_event_(boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this, _1)) {
    ros::NodeHandle cover_nh(nh, "cover");
    cover_command_sender_ = new rm_common::JointPositionBinaryCommandSender(cover_nh);
    XmlRpc::XmlRpcValue rpc_value;
    nh.getParam("cover_calibration", rpc_value);
    cover_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  }
  void run() override {
    ChassisGimbalShooterManual::run();
    cover_calibration_->update(ros::Time::now());
  }

 protected:
  void checkKeyboard() override {
    ChassisGimbalShooterManual::checkKeyboard();
    ctrl_z_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z);
  }
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalShooterManual::sendCommand(time);
    cover_command_sender_->sendCommand(time);
  }
  void shooterOutputOn(ros::Duration duration) override {
    ChassisGimbalShooterManual::shooterOutputOn(duration);
    cover_calibration_->reset();
  }
  void rightSwitchDown(ros::Duration duration) override {
    ChassisGimbalShooterManual::rightSwitchDown(duration);
    cover_command_sender_->open();
  }
  void rightSwitchMid(ros::Duration duration) override {
    ChassisGimbalShooterManual::rightSwitchMid(duration);
    cover_command_sender_->close();
  }
  void rightSwitchUp(ros::Duration duration) override {
    ChassisGimbalShooterManual::rightSwitchUp(duration);
    cover_command_sender_->close();
  }
  void mouseRightPress(ros::Duration duration) override {
    if (cover_command_sender_->getState())
      ChassisGimbalShooterManual::mouseRightPress(duration);
  }
  void mouseRightRelease(ros::Duration duration) override {
    if (cover_command_sender_->getState())
      ChassisGimbalShooterManual::mouseRightRelease(duration);
  }
  void ctrlZPress(ros::Duration /*duration*/) {
    if (data_.referee_.robot_id_ != rm_common::RobotId::BLUE_HERO
        && data_.referee_.robot_id_ != rm_common::RobotId::RED_HERO) {
      if (cover_command_sender_->getState()) {
        geometry_msgs::PointStamped aim_point{};
        aim_point.header.frame_id = "yaw";
        aim_point.header.stamp = ros::Time::now();
        aim_point.point.x = 1000;
        aim_point.point.y = 0;
        aim_point.point.z = 0;
        gimbal_cmd_sender_->setAimPoint(aim_point);
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
        cover_command_sender_->open();
      } else {
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        cover_command_sender_->close();
      }
    }
  }
  void drawUi() override {
    ChassisGimbalShooterManual::drawUi();
    state_ui_->update("cover", !cover_command_sender_->getState());
  }
  rm_common::JointPositionBinaryCommandSender *cover_command_sender_{};
  rm_common::CalibrationQueue *cover_calibration_;
  RisingInputEvent ctrl_z_press_event_;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_COVER_MANUAL_H_
