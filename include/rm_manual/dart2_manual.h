//
// Created by Aoalas on 2525/10/5.
//

#pragma once

#include "rm_manual/manual_base.h"
#include <rm_common/decision/calibration_queue.h>
#include <rm_msgs/DartClientCmd.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <unordered_map>
#include <rm_msgs/Dart.h>
#include <rm_msgs/DartInfo.h>

namespace rm_manual
{
class Dart2Manual : public ManualBase
{
public:
  Dart2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
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
    INIT = 0,
    PULLDOWN = 1,
    ENGAGE = 2,
    PULLUP = 3,
    READY = 4,
    PUSH = 5,
    RECYCLE = 6
  };
  enum AutoAimState
  {
    NONE,
    AIM,
    AIMED,
    ADJUST,
    ADJUSTED,
    FINISH
  };

  struct Dart
  {
    double outpost_offset_, base_offset_;
    double outpost_range_, base_range_;
  };

protected:
  void sendCommand(const ros::Time& time) override;
  void getList(const XmlRpc::XmlRpcValue& darts, const XmlRpc::XmlRpcValue& targets,
               const XmlRpc::XmlRpcValue& launch_id);
  void run() override;
  void checkReferee() override;
  void remoteControlTurnOn() override;
  void leftSwitchMidOn();
  void leftSwitchDownOn();
  void leftSwitchUpOn();
  void rightSwitchDownOn() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void move(rm_common::JointPointCommandSender* joint, double ch);
  void recordPosition(const rm_msgs::DbusData dbus_data);
  void readyLaunchDart(int dart_fired_num);
  bool triggerIsWorked() const;
  bool triggerIsHome() const;
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void dartInfoCallback(const rm_msgs::DartInfo::ConstPtr& data);
  void dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data);
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  void updateAllowDartDoorOpenTimes();
  void wheelClockwise();
  void wheelAntiClockwise();

  void load_dart(int step_idx);
  void load_work();
  void operateClamper(const rm_msgs::DbusData::ConstPtr& dbus_data);

  void updateLaunchMode();
  void init();
  void pullDown();
  void pullUp();
  void engage();
  void ready();
  void push();
  void recycle();

  void cameraDataCallback(const rm_msgs::Dart::ConstPtr& data);
  void updateAutoAimState();
  void aim();
  void adjust();
  void autoAim();
  bool shouldAutoAimCurrentDart() const;
  bool isAutoFireRestrictionEnabled() const;
  bool enemyOutpostAlive() const;

  int enemy_outpost_hp_{ 0 };
  int dart_current_target_;
  int dart_remaining_time_;
  bool enemy_outpost_dangerous_{ false };

  bool enemy_outpost_ignore_{ false };
  bool enemy_outpost_danger_detect_{ false };

  rm_common::JointPointCommandSender* yaw_sender_;
  rm_common::JointPointCommandSender* trigger_sender_;
  rm_common::JointPointCommandSender *belt_left_sender_, *belt_right_sender_;
  rm_common::JointPointCommandSender* range_sender_;
  rm_common::JointPointCommandSender* load_sender_;
  rm_common::JointPointCommandSender *clamp_left_sender_, *clamp_mid_sender_, *clamp_right_sender_;
  rm_common::CalibrationQueue *shooter_calibration_, *gimbal_calibration_, *clamp_calibration_;

  double range_outpost_{}, range_base_{}, yaw_outpost_{}, yaw_base_{};
  double belt_left_downward_vel_, belt_right_downward_vel_, belt_left_upward_vel_, belt_right_upward_vel_,
      belt_left_upward_slow_vel_, belt_right_upward_slow_vel_;
  std::unordered_map<int, Dart> dart_list_{};
  std::unordered_map<std::string, std::vector<double>> target_position_{};

  double scale_{ 0.05 }, scale_micro_{ 0.0005 };
  bool if_stop_{ true };

  uint8_t launch_mode_{ 0 }, last_launch_mode_{ 5 };
  uint8_t auto_aim_state_{ 0 }, last_auto_aim_state_{};

  rm_msgs::DbusData dbus_data_;
  uint8_t robot_id_, game_progress_{}, dart_launch_opening_status_{ 3 };
  uint16_t target_change_time_{ 0 };
  ros::Time gate_opened_time_{};
  ros::Time gate_opening_start_time_{};
  bool auto_shoot_active_{ false };
  int auto_shoot_dart_count_{ 0 };

  bool aborted_for_current_gate_{ false };
  bool has_entered_battle_{ false };
  uint8_t pc_last_game_progress_{};
  bool target_found_once_{ false };
  bool is_gate_actually_opening_{ false };
  ros::Time camera_lost_start_time_{};
  ros::Time pc_last_pulldown_time_{};

  uint16_t remain_time_{ 420 };

  int dart_fired_num_ = 0;
  double current_x_offset_ = 0.0;
  double target_x_offset = 0.0;
  double current_y_offset_ = 0.0;
  double target_y_offset_ = 0.0;
  double random_fixed_set_point_ = 0.0;
  double trigger_home_command_{}, trigger_work_command_{};
  double trigger_confirm_home_{}, trigger_confirm_work_{};
  double belt_left_position_{}, belt_right_position_{}, trigger_position_{};
  double belt_left_max_{}, belt_right_max_{}, belt_left_min_{}, belt_right_min_{}, belt_left_slow_{},
      belt_right_slow_{};
  double range_velocity_ = 0., yaw_velocity_ = 0., load_velocity_ = 0., load_position_ = 0.;

  double camera_x_set_point_, camera_y_set_point_;
  double camera_x_, camera_y_;
  double camera_fast_p_x_, camera_normal_p_x_, camera_slow_p_x_, camera_retarget_slow_p_x_;
  double camera_fast_x_threshold_, camera_normal_x_threshold_, camera_slow_x_threshold_;
  double camera_x_before_push_{}, camera_x_after_push_{};
  double camera_x_offset_, camera_y_offset_;
  double retarget_threshold_;

  bool is_camera_found_{ false };
  bool aim_failed_{ false };
  bool camera_x_init_{ false };

  bool camera_is_online_{};
  bool use_auto_aim_{}, start_aim_{}, auto_aim_start_{};
  bool random_fixed_target_{};
  bool random_fixed_compensation_{ false };
  bool get_reference_yaw_position_in_15s_{ false };
  bool custom_auto_aim_{ false };
  bool keep_auto_aim_effect_on_timeout_{ false };
  bool had_adjust_{};

  bool launch_result_{ false };

  bool retarget{ false };
  bool use_load_{ false }, load_start_{ false }, load_started_this_launch_{ false }, is_loading_{ false },
      load_finished_{ false }, clamp_finished_{ false };
  bool clamp_manual_{ true };
  bool belt_left_ready_{ false }, belt_right_ready_{ false }, dart_ready_{ false }, vision_ready_{ false };
  double clamp_left_release_position_, clamp_mid_release_position_, clamp_right_release_position_,
      clamp_default_position_;

  double load_init_position_, load_left_position_, load_mid_position_, load_right_position_;

  double current_load_target_{ 0.0 }, cur_pos{ 0.0 };

  InputEvent wheel_clockwise_event_, wheel_anticlockwise_event_;

  ros::Time last_engage_time_{};
  ros::Time last_push_time_{};
  ros::Time last_ready_time_{};
  ros::Time last_init_time_{};
  ros::Time last_get_camera_data_time_{};
  ros::Time pulldown_start_time_{};
  ros::Time pullup_start_time_{};
  ros::Time start_aim_time_{};
  ros::Time last_adjust_time_{};
  ros::Time last_clamp_finished_time_{};
  ros::Time load_start_time_{};
  ros::Time load_finish_time_{};
  ros::Time vision_end_time_{};
  ros::Time recycle_pulldown_start_time_{};
  ros::Time recycle_pullup_start_time_{};
  ros::Time recycle_start_time_{};
  ros::Time reference_yaw_15s_stage_start_time_{};
  ros::Time reference_yaw_15s_aim_start_time_{};

  ros::Subscriber dart_client_cmd_sub_;
  ros::Subscriber long_camera_data_sub_;
  ros::Subscriber camera_data_sub_;
  InputEvent dart_client_cmd_event_;

  int allow_dart_door_open_times_ = 0, last_dart_door_status_ = 1;

  int auto_state_ = BASE, manual_state_ = BASE, move_state_ = NORMAL;
  int clamp_num_{ 1 };
  int pc_once_fire_num_{ 4 }, all_pc_fired_num_{ 0 };
  int auto_fire_num_{ 4 }, auto_fire_count_{ 0 };

  bool temp_load_{ false };
  double temp_load_position_{ 0.01 };

  ros::Time push_failed_time = ros::Time::now();
  bool push_failed_ = true, push_succeeded_ = false, auto_push_ = false, auto_dart_ = false, load_skip_ = false;
  bool auto_fire_restriction_{ false };
  bool current_launch_auto_fire_{ false }, current_launch_auto_fire_counted_{ false };
  bool pulldown_timeout_detect_{ false }, pullup_timeout_detect_{ false };
  bool recycle_pulldown_timeout_detect_{ false };
  bool recycle_timeout_detect_{ false };
  double pulldown_timeout_s_{ 6.0 }, pullup_timeout_s_{ 6.0 };

  bool left_recycle_done = false, right_recycle_done = false;
  bool dart_recycling_ = false, recycle_mode_ = false, pc_target_detection_ = true, recycle_should_skip_load_ = false;
  uint8_t recycle_step_{ 0 };
  ros::Time recycle_timer_{};

  double false_engage_time_{ 1.0 };
  double random_fixed_reference_yaw_position_{ 0.0 };
  double random_fixed_compensation_coefficient_{ 0.0 };

  bool referee_data_okay_{ true };

  ros::Time camera_target_visible_start_{};
  ros::Time camera_target_lost_start_{};
  ros::Time fake_gate_opened_time_{};
  bool fake_gate_is_open_{ false };

  bool reference_yaw_15s_started_{ false };
  bool reference_yaw_15s_aiming_{ false };
  bool reference_yaw_15s_finished_{ false };
  bool reference_yaw_15s_success_{ false };
  double reference_yaw_15s_set_point_{ 0.0 };

  bool test_auto_aim_active_{ false };
  bool test_camera_online_active_{ false };
  bool load_init_toggle_active_{ false };
  double test_yaw_start_pos_;
  bool s_r_was_up_{ false }, s_l_was_down_{ true };

  bool pc_locked_{ false }, reset_belt_{ false };

  std::vector<bool> trigger_status_;
  std::vector<bool> auto_aim_status_{ true, true, true, true };
};

}  // namespace rm_manual
