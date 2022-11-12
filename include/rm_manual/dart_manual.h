//
// Created by luotinkai on 2022/7/15.
//

#pragma once

#include "rm_manual/common/manual_base.h"
#include <rm_common/decision/calibration_queue.h>

namespace rm_manual
{
class DartManual : public ManualBase
{
public:
  explicit DartManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void sendCommand(const ros::Time& time) override;
  void run() override;
  void checkReferee() override;
  void remoteControlTurnOn() override;
  void leftSwitchUpFall();
  void leftSwitchDownRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchUpRise() override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void move(rm_common::JointPointCommandSender* joint, double ch);
  void recordPosition(const rm_msgs::DbusData::ConstPtr& dbus_data);

  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override
  {
    ManualBase::gameRobotStatusCallback(data);
    chassis_power_on_event_.update(data->mains_power_chassis_output);
    gimbal_power_on_event_.update(data->mains_power_gimbal_output);
  }

  rm_common::JointPointCommandSender *trigger_sender_, *friction_left_sender_, *friction_right_sender_;
  rm_common::JointPointCommandSender *pitch_sender_, *yaw_sender_;
  rm_common::CalibrationQueue *trigger_calibration_, *gimbal_calibration_;
  double pitch_outpost_{}, pitch_base_{}, yaw_outpost_{}, yaw_base_{};
  double qd_, upward_vel_;
  double scale_{ 0.04 };
  bool if_stop_{ true };

  InputEvent chassis_power_on_event_, gimbal_power_on_event_;
};
}  // namespace rm_manual
