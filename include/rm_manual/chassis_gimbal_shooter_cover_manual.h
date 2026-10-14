//
// Created by chenzheng on 7/20/21.
//

#pragma once

#include "rm_manual/chassis_gimbal_shooter_manual.h"

#include <std_msgs/Float64.h>
#include <algorithm>
#include <rm_common/math_utilities.h>
#include <control_toolbox/pid.h>

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
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkReferee() override;
  void sendCommand(const ros::Time& time) override;
  void remoteControlTurnOn() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchDownRise() override;
  void mouseRightPress() override;
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
  void rPress() override;
  void bRelease() override
  {
  }
  void zPress() override;
  void zRelease();
  void ctrlCPress() override
  {
  }
  void shiftPress() override;
  virtual void rRelease();
  virtual void ctrlZPress();
  virtual void ctrlZRelease();
  virtual void ctrlXPress();

  void getPitchErr(double& position);
  void gimbalOutputOn() override;

  double low_speed_scale_{}, normal_speed_scale_{};
  double exit_buff_mode_duration_{};
  double gyro_speed_limit_{};
  double sin_gyro_base_scale_{ 1. }, sin_gyro_amplitude_{ 0. }, sin_gyro_period_{ 1. }, sin_gyro_phase_{ 0. };

  ros::Publisher base_pitch_pub_;

  rm_common::SwitchDetectionCaller* switch_buff_srv_{};
  rm_common::SwitchDetectionCaller* switch_buff_type_srv_{};
  rm_common::SwitchDetectionCaller* switch_exposure_srv_{};

  InputEvent ctrl_z_event_;

  std::string wireless_frame_;
  ros::Time last_switch_time_;
  ros::Time gimbal_output_on_time_;
  bool need_wireless_{ false };
  int count_{};

  bool zipped_{ false };
  std::shared_ptr<control_toolbox::Pid> zipped_pitch_rate_pid_{};
};
}  // namespace rm_manual
