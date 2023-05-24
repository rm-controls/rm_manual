//
// Created by chenzheng on 7/20/21.
//

#pragma once

#include "rm_manual/chassis_gimbal_shooter_manual.h"

namespace rm_manual
{
class ChassisGimbalShooterCoverManual : public ChassisGimbalShooterManual
{
public:
  ChassisGimbalShooterCoverManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

protected:
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkReferee() override;
  void sendCommand(const ros::Time& time) override;
  void gimbalOutputOn() override;
  void remoteControlTurnOff() override;
  void remoteControlTurnOn() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void rPress() override;
  void ePress() override;
  virtual void ctrlZPress();
  virtual void ctrlZRelease()
  {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  };
  virtual void ctrlQPress();
  rm_common::SwitchDetectionCaller* switch_buff_srv_{};
  rm_common::SwitchDetectionCaller* switch_buff_type_srv_{};
  rm_common::JointPositionBinaryCommandSender* cover_command_sender_{};
  rm_common::CalibrationQueue* gimbal_calibration_;
  InputEvent ctrl_z_event_, ctrl_q_event_, x_event_, z_event_;
  std::string supply_frame_;
  bool supply_ = false;
  bool cover_close_ = true;
};
}  // namespace rm_manual
