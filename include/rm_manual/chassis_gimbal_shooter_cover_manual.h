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
  enum SpeedMode
  {
    LOW,
    NORMAL
  };

protected:
  void changeSpeedMode(SpeedMode speed_mode);
  double getDynamicScale(const double base_scale, const double amplitude, const double period, const double phase);
  void changeGyroSpeedMode(SpeedMode speed_mode);
  void check_wheel_online();
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkReferee() override;
  void sendCommand(const ros::Time& time) override;
  void updateWheelState(const rm_ecat_msgs::RmEcatStandardSlaveReadings::ConstPtr& data,
                        const std::vector<std::string>& wheel_vector);
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void wheelOnlineCallback(const rm_ecat_msgs::RmEcatStandardSlaveReadings::ConstPtr& data);
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void ePress() override;
  void eRelease() override;
  void bPress() override;
  void cPress() override;
  void qPress() override;
  void ctrlRPressing();
  void ctrlRRelease() override;
  void wPress() override;
  void wPressing() override;
  void aPressing() override;
  void sPressing() override;
  void dPressing() override;
  void wRelease() override;
  void aRelease() override;
  void sRelease() override;
  void dRelease() override;
  void zPress() override
  {
  }

  virtual void ctrlZPress();
  virtual void ctrlZRelease()
  {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  };

  ros::Subscriber wheel_online_sub_;

  double low_speed_scale_{}, normal_speed_scale_{};
  double exit_buff_mode_duration_{};
  double gyro_speed_limit_{};
  double sin_gyro_base_scale_{ 1. }, sin_gyro_amplitude_{ 0. }, sin_gyro_period_{ 1. }, sin_gyro_phase_{ 0. };
  rm_common::SwitchDetectionCaller* switch_buff_srv_{};
  rm_common::SwitchDetectionCaller* switch_buff_type_srv_{};
  rm_common::SwitchDetectionCaller* switch_exposure_srv_{};
  rm_common::JointPositionBinaryCommandSender* cover_command_sender_{};
  InputEvent ctrl_z_event_, z_event_;
  std::string supply_frame_;
  ros::Time last_switch_time_, last_check_wheel_time_;
  std::vector<std::string> chassis_motor_;
  std::vector<bool> wheel_online_state_;
  bool supply_ = false;
  bool cover_close_ = true;
  bool last_power_chassis_output_ = false;
  int count_{};
};
}  // namespace rm_manual
