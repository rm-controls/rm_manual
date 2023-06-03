//
// Created by luotinkai on 2022/7/15.
//

#pragma once

#include "rm_manual/manual_base.h"
#include <rm_common/decision/calibration_queue.h>
#include <rm_msgs/DartClientCmd.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>

namespace rm_manual
{
class DartManual : public ManualBase
{
public:
  DartManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  enum AimMode
  {
    OUTPOST,
    BASE
  };
  enum MoveMode
  {
    NORMAL,
    MICRO,
    MOVING,
    STOP
  };
  enum LaunchMode
  {
    NONE,
    FIRST_OUTPOST,
    SECOND_OUTPOST,
    ALL_BASE
  };

protected:
  void sendCommand(const ros::Time& time) override;
  void run() override;
  void checkReferee() override;
  void remoteControlTurnOn() override;
  void leftSwitchMidOn();
  void leftSwitchDownOn();
  void leftSwitchUpOn();
  void rightSwitchDownOn();
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void move(rm_common::JointPointCommandSender* joint, double ch);
  void recordPosition(const rm_msgs::DbusData dbus_data);
  void launchTwoDart();
  void getDartFiredNum();
  void triggerComeBackProtect();
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data);
  void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data) override;
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  void wheelClockwise();
  void wheelAntiClockwise();
  rm_common::JointPointCommandSender *trigger_sender_, *friction_left_sender_, *friction_right_sender_;
  rm_common::JointPointCommandSender *pitch_sender_, *yaw_sender_;
  rm_common::CalibrationQueue *trigger_calibration_, *gimbal_calibration_;
  double pitch_outpost_{}, pitch_base_{}, yaw_outpost_{}, yaw_base_{};
  double pitch_position_outpost_{}, yaw_position_outpost_{}, pitch_position_base_{}, yaw_position_base_{};
  double qd_, upward_vel_;
  std::vector<double> qd_outpost_, qd_base_, yaw_offset_, yaw_offset_base_;
  double scale_{ 0.04 }, scale_micro_{ 0.01 };
  bool if_stop_{ true };

  bool launch_rest_flag_ = 0, has_launched_ = 0;
  rm_msgs::DbusData dbus_data_;
  rm_msgs::DartClientCmd dart_client_cmd_;
  rm_msgs::GameRobotStatus game_robot_status_;
  rm_msgs::GameStatus game_status_;

  int dart_fired_num_ = 0;
  double launch_position_1_ = 0.003251, launch_position_2_ = 0.008298, launch_position_3_ = 0.014457;
  double trigger_position_ = 0., pitch_velocity_ = 0., yaw_velocity_ = 0.;
  InputEvent wheel_clockwise_event_, wheel_anticlockwise_event_;

  ros::Subscriber dart_client_cmd_sub_;
  InputEvent dart_client_cmd_event_;
  int outpost_hp_;
  int dart_door_open_times_ = 0, last_dart_door_status_ = 1;
  int auto_state_ = OUTPOST, manual_state_ = OUTPOST, move_state_ = NORMAL, launch_state_ = FIRST_OUTPOST,
      last_launch_state_ = FIRST_OUTPOST;
};
}  // namespace rm_manual
