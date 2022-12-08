//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include "rm_manual/common/manual_base.h"

namespace rm_manual
{
class ChassisGimbalManual : public ManualBase
{
public:
  explicit ChassisGimbalManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void sendCommand(const ros::Time& time) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void checkReferee() override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void remoteControlTurnOff() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchMidFall() override;
  void leftSwitchDownRise() override;
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override
  {
    ManualBase::gameStatusCallback(data);
    chassis_cmd_sender_->updateGameStatus(*data);
  }
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override
  {
    ManualBase::gameRobotStatusCallback(data);
    chassis_cmd_sender_->updateGameRobotStatus(*data);
    chassis_power_on_event_.update(data->mains_power_chassis_output);
    gimbal_power_on_event_.update(data->mains_power_gimbal_output);
  }
  void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data) override
  {
    ManualBase::powerHeatDataCallback(data);
    chassis_cmd_sender_->updatePowerHeatData(*data);
  }
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override
  {
    ManualBase::dbusDataCallback(data);
    chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
  }
  void capacityDataCallback(const rm_msgs::CapacityData ::ConstPtr& data) override
  {
    ManualBase::capacityDataCallback(data);
    chassis_cmd_sender_->updateCapacityData(*data);
  }
  void trackCallback(const rm_msgs::TrackData::ConstPtr& data) override
  {
    ManualBase::trackCallback(data);
  }
  virtual void wPress()
  {
    x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  }
  virtual void wRelease();
  virtual void sPress()
  {
    x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  }
  virtual void sRelease();
  virtual void aPress()
  {
    y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  }
  virtual void aRelease();
  virtual void dPress()
  {
    y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  }
  virtual void dRelease();
  void wPressing();
  void aPressing();
  void sPressing();
  void dPressing();
  void mouseMidRise();

  rm_common::Vel2DCommandSender* vel_cmd_sender_{};
  rm_common::GimbalCommandSender* gimbal_cmd_sender_{};
  rm_common::ChassisCommandSender* chassis_cmd_sender_{};
  double x_scale_{}, y_scale_{};
  double gimbal_scale_{ 1. };
  double gyro_move_reduction_{ 1. };
  double gyro_rotate_reduction_{ 1. };

  InputEvent chassis_power_on_event_, gimbal_power_on_event_, w_event_, s_event_, a_event_, d_event_, mouse_mid_event_;
};
}  // namespace rm_manual
