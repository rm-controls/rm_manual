//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include "rm_manual/chassis_gimbal_manual.h"
#include <rm_common/decision/calibration_queue.h>

namespace rm_manual
{
class ChassisGimbalShooterManual : public ChassisGimbalManual
{
public:
  explicit ChassisGimbalShooterManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

protected:
  void checkReferee() override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void sendCommand(const ros::Time& time) override;
  void chassisOutputOn() override;
  void shooterOutputOn() override;
  void selfInspectionStart()
  {
    shooter_calibration_->reset();
  };
  void gameStart()
  {
    shooter_calibration_->reset();
  };
  void remoteControlTurnOff() override;
  void remoteControlTurnOn() override;
  void robotDie() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchDownRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchMidOn(ros::Duration duration);
  void leftSwitchUpRise() override;
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override
  {
    ChassisGimbalManual::gameRobotStatusCallback(data);
    shooter_cmd_sender_->updateGameRobotStatus(*data);
    shooter_power_on_event_.update(data->mains_power_shooter_output);
  }
  void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data) override
  {
    ChassisGimbalManual::powerHeatDataCallback(data);
    shooter_cmd_sender_->updatePowerHeatData(*data);
  }
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override
  {
    ChassisGimbalManual::dbusDataCallback(data);
    chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
    shooter_cmd_sender_->updateRefereeStatus(referee_is_online_);
  }
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override
  {
    ChassisGimbalManual::gameStatusCallback(data);
    self_inspection_event_.update(data->game_progress == 2);
    game_start_event_.update(data->game_progress == 4);
  }
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data) override
  {
    ChassisGimbalManual::gimbalDesErrorCallback(data);
  }
  void leftSwitchUpOn(ros::Duration duration);
  void mouseLeftPress();
  void mouseLeftRelease()
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
  void mouseRightPress();
  void mouseRightRelease()
  {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  void wPress() override;
  void aPress() override;
  void sPress() override;
  void dPress() override;
  void ePress();
  void cPress();
  void bPress();
  void qPress()
  {
    if (shooter_cmd_sender_->getShootFrequency() != rm_common::HeatLimit::LOW)
      shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::LOW);
    else
      shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::BURST);
  }
  void fPress()
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void shiftPress();
  void shiftRelease();
  void ctrlCPress();
  void ctrlVPress();
  void ctrlRPress();
  void ctrlBPress();

  InputEvent shooter_power_on_event_, self_inspection_event_, game_start_event_, e_event_, c_event_, g_event_, q_event_,
      f_event_, b_event_, x_event_, ctrl_c_event_, ctrl_v_event_, ctrl_r_event_, ctrl_b_event_, shift_event_,
      ctrl_shift_b_event_, mouse_left_event_, mouse_right_event_;
  rm_common::ShooterCommandSender* shooter_cmd_sender_{};
  rm_common::SwitchDetectionCaller* switch_detection_srv_{};
  rm_common::CalibrationQueue* shooter_calibration_;
};
}  // namespace rm_manual
