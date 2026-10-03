//
// Created by Aoalas on 2525/10/5.
//

#include "rm_manual/dart2_manual.h"

namespace rm_manual
{
Dart2Manual::Dart2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  XmlRpc::XmlRpcValue dart_list, targets, launch_id, steps_xml, trigger_status_rpc, auto_aim_status_rpc;
  nh.getParam("launch_id", launch_id);
  nh.getParam("dart_list", dart_list);
  nh.getParam("targets", targets);
  getList(dart_list, targets, launch_id);

  yaw_outpost_ = target_position_["outpost"][0];
  range_outpost_ = target_position_["outpost"][1];
  yaw_base_ = target_position_["base"][0];
  range_base_ = target_position_["base"][1];

  ros::NodeHandle nh_yaw = ros::NodeHandle(nh, "yaw");
  yaw_sender_ = new rm_common::JointPointCommandSender(nh_yaw, joint_state_);
  ros::NodeHandle nh_trigger = ros::NodeHandle(nh, "trigger");
  trigger_sender_ = new rm_common::JointPointCommandSender(nh_trigger, joint_state_);
  ros::NodeHandle nh_load = ros::NodeHandle(nh, "load");
  load_sender_ = new rm_common::JointPointCommandSender(nh_load, joint_state_);

  nh_load.getParam("use_load", use_load_);
  nh_load.getParam("load_init_position", load_init_position_);
  nh_load.getParam("load_left_position", load_left_position_);
  nh_load.getParam("load_mid_position", load_mid_position_);
  nh_load.getParam("load_right_position", load_right_position_);

  nh_trigger.getParam("trigger_home_command", trigger_home_command_);
  nh_trigger.getParam("trigger_work_command", trigger_work_command_);
  nh_trigger.getParam("trigger_confirm_home", trigger_confirm_home_);
  nh_trigger.getParam("trigger_confirm_work", trigger_confirm_work_);
  nh_trigger.getParam("false_engage_time", false_engage_time_);

  nh_trigger.getParam("auto_push", auto_push_);
  nh_trigger.getParam("auto_dart", auto_dart_);
  nh_trigger.param("auto_fire_restriction", auto_fire_restriction_, false);
  nh_trigger.param("auto_fire_num", auto_fire_num_, 4);

  nh_trigger.param("pulldown_timeout_detect", pulldown_timeout_detect_, false);
  nh_trigger.param("pulldown_timeout_s", pulldown_timeout_s_, 6.0);
  nh_trigger.param("pullup_timeout_detect", pullup_timeout_detect_, false);
  nh_trigger.param("pullup_timeout_s", pullup_timeout_s_, 6.0);
  nh_trigger.param("recycle_pulldown_timeout_detect", recycle_pulldown_timeout_detect_, false);
  nh_trigger.param("recycle_timeout_detect", recycle_timeout_detect_, false);
  nh_trigger.getParam("recycle_mode", recycle_mode_);

  nh_trigger.getParam("pc_target_detection", pc_target_detection_);
  nh_trigger.getParam("pc_once_fire_num", pc_once_fire_num_);
  nh_trigger.getParam("referee_data_okay", referee_data_okay_);

  nh_trigger.param("enemy_outpost_ignore", enemy_outpost_ignore_, false);
  nh_trigger.param("enemy_outpost_danger_detect", enemy_outpost_danger_detect_, false);

  if (auto_fire_num_ < 0)
  {
    ROS_WARN("auto_fire_num is %d, expected non-negative. Initialize to 0.", auto_fire_num_);
    auto_fire_num_ = 0;
  }
  if (pulldown_timeout_s_ < 0.0)
  {
    ROS_WARN("pulldown_timeout_s is %f, expected non-negative. Initialize to 0.0.", pulldown_timeout_s_);
    pulldown_timeout_s_ = 0.0;
  }
  if (pullup_timeout_s_ < 0.0)
  {
    ROS_WARN("pullup_timeout_s is %f, expected non-negative. Initialize to 0.0.", pullup_timeout_s_);
    pullup_timeout_s_ = 0.0;
  }

  if (nh_trigger.getParam("trigger_status", trigger_status_rpc))
  {
    ROS_ASSERT(trigger_status_rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < trigger_status_rpc.size(); ++i)
    {
      trigger_status_.push_back(static_cast<bool>(trigger_status_rpc[i]));
    }
  }
  else
  {
    trigger_status_ = { true, true, true, true };
  }

  ros::NodeHandle nh_clamp = ros::NodeHandle(nh, "clamp_positions");
  nh_clamp.getParam("clamp_left_release_position", clamp_left_release_position_);
  nh_clamp.getParam("clamp_mid_release_position", clamp_mid_release_position_);
  nh_clamp.getParam("clamp_right_release_position", clamp_right_release_position_);
  nh_clamp.getParam("clamp_default_position", clamp_default_position_);
  nh_clamp.param("clamp_manual", clamp_manual_, true);

  ros::NodeHandle nh_belt_left = ros::NodeHandle(nh, "belt_left");
  ros::NodeHandle nh_belt_right = ros::NodeHandle(nh, "belt_right");
  belt_left_sender_ = new rm_common::JointPointCommandSender(nh_belt_left, joint_state_);
  belt_right_sender_ = new rm_common::JointPointCommandSender(nh_belt_right, joint_state_);
  nh_belt_left.getParam("belt_left_max", belt_left_max_);
  nh_belt_right.getParam("belt_right_max", belt_right_max_);
  nh_belt_left.getParam("belt_left_slow", belt_left_slow_);
  nh_belt_right.getParam("belt_right_slow", belt_right_slow_);
  nh_belt_left.getParam("belt_left_min", belt_left_min_);
  nh_belt_right.getParam("belt_right_min", belt_right_min_);
  nh_belt_left.getParam("belt_left_downward_vel", belt_left_downward_vel_);
  nh_belt_right.getParam("belt_right_downward_vel", belt_right_downward_vel_);
  nh_belt_left.getParam("belt_left_upward_vel", belt_left_upward_vel_);
  nh_belt_right.getParam("belt_right_upward_vel", belt_right_upward_vel_);
  nh_belt_left.getParam("belt_left_upward_slow_vel", belt_left_upward_slow_vel_);
  nh_belt_right.getParam("belt_right_upward_slow_vel", belt_right_upward_slow_vel_);

  ros::NodeHandle nh_range = ros::NodeHandle(nh, "range");
  range_sender_ = new rm_common::JointPointCommandSender(nh_range, joint_state_);
  nh_range.param("random_fixed_compensation", random_fixed_compensation_, false);
  nh_range.param("get_reference_yaw_position_in_15s", get_reference_yaw_position_in_15s_, false);
  nh_range.param("random_fixed_reference_yaw_position", random_fixed_reference_yaw_position_, 0.032836);
  nh_range.param("random_fixed_compensation_coefficient", random_fixed_compensation_coefficient_, -0.11);

  ros::NodeHandle nh_clamp_left = ros::NodeHandle(nh, "clamp_left");
  ros::NodeHandle nh_clamp_mid = ros::NodeHandle(nh, "clamp_mid");
  ros::NodeHandle nh_clamp_right = ros::NodeHandle(nh, "clamp_right");
  clamp_left_sender_ = new rm_common::JointPointCommandSender(nh_clamp_left, joint_state_);
  clamp_mid_sender_ = new rm_common::JointPointCommandSender(nh_clamp_mid, joint_state_);
  clamp_right_sender_ = new rm_common::JointPointCommandSender(nh_clamp_right, joint_state_);

  XmlRpc::XmlRpcValue shooter_rpc_value, gimbal_rpc_value, clamp_rpc_value;

  nh.getParam("shooter_calibration", shooter_rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(shooter_rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", gimbal_rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_rpc_value, nh, controller_manager_);
  nh.getParam("clamp_calibration", clamp_rpc_value);
  clamp_calibration_ = new rm_common::CalibrationQueue(clamp_rpc_value, nh, controller_manager_);

  ros::NodeHandle nh_camera = ros::NodeHandle(nh, "camera");
  nh_camera.getParam("camera_x_offset", camera_x_offset_);
  nh_camera.getParam("camera_y_offset", camera_y_offset_);

  nh_camera.getParam("camera_fast_p_x", camera_fast_p_x_);
  nh_camera.getParam("camera_normal_p_x", camera_normal_p_x_);
  nh_camera.getParam("camera_slow_p_x", camera_slow_p_x_);
  nh_camera.getParam("camera_retarget_slow_p_x", camera_retarget_slow_p_x_);
  nh_camera.getParam("camera_fast_x_threshold", camera_fast_x_threshold_);
  nh_camera.getParam("camera_normal_x_threshold", camera_normal_x_threshold_);
  nh_camera.getParam("camera_slow_x_threshold", camera_slow_x_threshold_);
  nh_camera.getParam("retarget_threshold", retarget_threshold_);

  nh_camera.getParam("use_auto_aim", use_auto_aim_);
  nh_camera.param("keep_auto_aim_effect_on_timeout", keep_auto_aim_effect_on_timeout_, false);
  nh_camera.getParam("random_fixed_target", random_fixed_target_);
  nh_camera.param("custom_auto_aim", custom_auto_aim_, false);

  if (nh_camera.getParam("auto_aim_status", auto_aim_status_rpc))
  {
    ROS_ASSERT(auto_aim_status_rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);
    auto_aim_status_.clear();
    for (int i = 0; i < auto_aim_status_rpc.size(); ++i)
    {
      auto_aim_status_.push_back(static_cast<bool>(auto_aim_status_rpc[i]));
    }
  }

  if (auto_aim_status_.size() != 4)
  {
    ROS_WARN("auto_aim_status size is %zu, expected 4. Falling back to [true, true, true, true].",
             auto_aim_status_.size());
    auto_aim_status_ = { true, true, true, true };
  }

  left_switch_up_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchUpOn, this));
  left_switch_mid_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchMidOn, this));
  left_switch_down_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchDownOn, this));
  right_switch_down_event_.setActiveHigh(boost::bind(&Dart2Manual::rightSwitchDownOn, this));
  right_switch_mid_event_.setRising(boost::bind(&Dart2Manual::rightSwitchMidRise, this));
  right_switch_up_event_.setRising(boost::bind(&Dart2Manual::rightSwitchUpRise, this));
  wheel_clockwise_event_.setRising(boost::bind(&Dart2Manual::wheelClockwise, this));
  wheel_anticlockwise_event_.setRising(boost::bind(&Dart2Manual::wheelAntiClockwise, this));
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/rm_ecat_hw/dbus", 10, &Dart2Manual::dbusDataCallback, this);
  dart_client_cmd_sub_ = nh_referee.subscribe<rm_msgs::DartClientCmd>("dart_client_cmd_data", 10,
                                                                      &Dart2Manual::dartClientCmdCallback, this);

  game_robot_hp_sub_ =
      nh_referee.subscribe<rm_msgs::GameRobotHp>("game_robot_hp", 10, &Dart2Manual::gameRobotHpCallback, this);

  game_status_sub_ =
      nh_referee.subscribe<rm_msgs::GameStatus>("game_status", 10, &Dart2Manual::gameStatusCallback, this);

  camera_data_sub_ =
      nh.subscribe<rm_msgs::Dart>("/short_dart_camera_distance", 10, &Dart2Manual::cameraDataCallback, this);
}

void Dart2Manual::getList(const XmlRpc::XmlRpcValue& darts, const XmlRpc::XmlRpcValue& targets,
                          const XmlRpc::XmlRpcValue& launch_id)
{
  for (const auto& dart : darts)
  {
    ROS_ASSERT(dart.second.hasMember("param") and dart.second.hasMember("id"));
    ROS_ASSERT(dart.second["param"].getType() == XmlRpc::XmlRpcValue::TypeArray and
               dart.second["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    for (int i = 0; i < 4; ++i)
    {
      if (dart.second["id"] == launch_id[i])
      {
        Dart dart_info;
        dart_info.outpost_offset_ = static_cast<double>(dart.second["param"][0]);
        dart_info.outpost_range_ = static_cast<double>(dart.second["param"][1]);
        dart_info.base_offset_ = static_cast<double>(dart.second["param"][2]);
        dart_info.base_range_ = static_cast<double>(dart.second["param"][3]);
        dart_list_.insert(std::make_pair(i, dart_info));
      }
    }
  }

  for (const auto& target : targets)
  {
    ROS_ASSERT(target.second.hasMember("position"));
    ROS_ASSERT(target.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<double> position(2);
    position[0] = static_cast<double>(target.second["position"][0]);
    position[1] = static_cast<double>(target.second["position"][1]);
    target_position_.insert(std::make_pair(target.first, position));
  }
}

void Dart2Manual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  robot_id_ = data->robot_id;
}

void Dart2Manual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  gimbal_calibration_->stopController();
  gimbal_calibration_->reset();
  shooter_calibration_->stopController();
  shooter_calibration_->reset();
  clamp_calibration_->stopController();
  clamp_calibration_->reset();
}

void Dart2Manual::checkReferee()
{
  ManualBase::checkReferee();
}

void Dart2Manual::run()
{
  ManualBase::run();
  gimbal_calibration_->update(ros::Time::now());
  shooter_calibration_->update(ros::Time::now());
  clamp_calibration_->update(ros::Time::now());
  updateLaunchMode();
  updateAutoAimState();
}

void Dart2Manual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  const uint8_t pc_prev_game_progress = pc_last_game_progress_;
  bool pc_was_in_battle = (pc_last_game_progress_ == rm_msgs::GameStatus::IN_BATTLE);

  if (game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
  {
    has_entered_battle_ = true;
  }
  else if (has_entered_battle_ && pc_was_in_battle)
  {
    auto_shoot_active_ = false;
    aborted_for_current_gate_ = true;
    target_found_once_ = false;
    camera_lost_start_time_ = ros::Time(0);
    range_sender_->setPoint(0.02);

    size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
    bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;
    if (recycle_mode_ && is_dart_active && (launch_mode_ == ENGAGE || launch_mode_ == PULLUP || launch_mode_ == READY))
    {
      recycle_pulldown_start_time_ = ros::Time(0);
      recycle_pullup_start_time_ = ros::Time(0);
      recycle_start_time_ = ros::Time(0);
      dart_recycling_ = true;
      launch_mode_ = RECYCLE;
      ROS_INFO("Battle finished during dart charging. Enter RECYCLE.");
    }
    else if (launch_mode_ != RECYCLE)
    {
      last_init_time_ = ros::Time::now();
      launch_mode_ = INIT;
    }
  }
  if (pc_prev_game_progress == 2 && game_progress_ == 3 && reference_yaw_15s_success_)
  {
    double default_yaw_position = random_fixed_reference_yaw_position_;
    switch (manual_state_)
    {
      case OUTPOST:
        default_yaw_position = target_position_["outpost"][0];
        break;
      case BASE:
        default_yaw_position = target_position_["base"][0];
        break;
      default:
        default_yaw_position = target_position_["base"][0];
        break;
    }
    yaw_sender_->setPoint(default_yaw_position);
    ROS_INFO("Restore yaw to default target after 15s prepare stage: %lf", default_yaw_position);
  }

  if (game_progress_ == 2)
  {
    if (get_reference_yaw_position_in_15s_ && !reference_yaw_15s_finished_)
    {
      const ros::Time now = ros::Time::now();

      if (!reference_yaw_15s_started_)
      {
        reference_yaw_15s_started_ = true;
        reference_yaw_15s_success_ = false;
        reference_yaw_15s_stage_start_time_ = now;
        reference_yaw_15s_aim_start_time_ = ros::Time(0);
      }

      if (now - reference_yaw_15s_stage_start_time_ > ros::Duration(5.0))
      {
        if (!reference_yaw_15s_aiming_)
        {
          int yaw_idx = yaw_sender_->getIndex();
          reference_yaw_15s_set_point_ = (yaw_idx != -1 && yaw_idx < static_cast<int>(joint_state_.position.size())) ?
                                             joint_state_.position[yaw_idx] :
                                             yaw_base_;
          reference_yaw_15s_aiming_ = true;
          reference_yaw_15s_aim_start_time_ = now;
          ROS_INFO("Start getting reference yaw position in 15s preparation stage.");
        }

        if (reference_yaw_15s_aiming_)
        {
          bool camera_15s_online = (now - last_get_camera_data_time_ < ros::Duration(1.0));

          if (now - reference_yaw_15s_aim_start_time_ > ros::Duration(6.0))
          {
            reference_yaw_15s_aiming_ = false;
            reference_yaw_15s_finished_ = true;
            ROS_INFO("Get reference yaw position in 15s preparation stage timeout.");
          }
          else
          {
            if (camera_15s_online && is_camera_found_ && std::abs(camera_x_) >= camera_fast_x_threshold_)
            {
              reference_yaw_15s_set_point_ += camera_x_ * camera_fast_p_x_;
            }
            if (camera_15s_online && is_camera_found_ && std::abs(camera_x_) < camera_fast_x_threshold_ &&
                std::abs(camera_x_) >= camera_normal_x_threshold_)
            {
              reference_yaw_15s_set_point_ += camera_x_ * camera_normal_p_x_;
            }
            if (camera_15s_online && is_camera_found_ && std::abs(camera_x_) < camera_normal_x_threshold_ &&
                std::abs(camera_x_) >= camera_slow_x_threshold_)
            {
              reference_yaw_15s_set_point_ += camera_x_ * (retarget ? camera_retarget_slow_p_x_ : camera_slow_p_x_);
            }

            yaw_sender_->setPoint(reference_yaw_15s_set_point_);

            if (camera_15s_online && is_camera_found_ && std::abs(camera_x_) <= camera_slow_x_threshold_ &&
                yaw_velocity_ < 0.01)
            {
              int yaw_idx = yaw_sender_->getIndex();
              random_fixed_reference_yaw_position_ =
                  (yaw_idx != -1 && yaw_idx < static_cast<int>(joint_state_.position.size())) ?
                      joint_state_.position[yaw_idx] :
                      reference_yaw_15s_set_point_;
              reference_yaw_15s_aiming_ = false;
              reference_yaw_15s_finished_ = true;
              reference_yaw_15s_success_ = true;
              ROS_INFO("Reference yaw position in 15s preparation stage: %lf", random_fixed_reference_yaw_position_);
            }
          }
        }
      }
    }
  }
  else
  {
    reference_yaw_15s_started_ = false;
    reference_yaw_15s_aiming_ = false;
    reference_yaw_15s_finished_ = false;
    reference_yaw_15s_success_ = false;
    reference_yaw_15s_stage_start_time_ = ros::Time(0);
    reference_yaw_15s_aim_start_time_ = ros::Time(0);
  }

  pc_last_game_progress_ = game_progress_;

  if (all_pc_fired_num_ >= 4)
  {
    ROS_INFO_THROTTLE(5.0, "All 4 darts fired! PC mode locked. Waiting for Right Switch MID to reset.");

    if (ros::Time::now() - last_push_time_ > ros::Duration(2.0))
    {
      range_sender_->setPoint(0.02);
    }

    if (std::abs(range_velocity_) < 0.01 && std::abs(yaw_velocity_) < 0.01)
    {
      if (ros::Time::now() - last_push_time_ > ros::Duration(5.0) && reset_belt_)
      {
        belt_left_sender_->setPoint(5.0);
        belt_right_sender_->setPoint(5.0);
      }
      if (reset_belt_ && (belt_left_position_ >= 15.0 || belt_right_position_ >= 15.0))
      {
        belt_left_sender_->setPoint(-5.0);
        belt_right_sender_->setPoint(-5.0);
        reset_belt_ = false;
      }
      if (!reset_belt_ && (belt_left_position_ <= belt_left_min_ || belt_right_position_ <= belt_right_min_))
      {
        belt_left_sender_->setPoint(0.0);
        belt_right_sender_->setPoint(0.0);
      }
    }

    auto_shoot_active_ = false;
    aborted_for_current_gate_ = true;
    pc_locked_ = true;

    if (dart_recycling_)
    {
      readyLaunchDart(dart_fired_num_);
    }
    else if (launch_mode_ != INIT)
    {
      launch_mode_ = INIT;
    }
    return;
  }

  if (referee_data_okay_ && !pc_locked_)
  {
    ROS_INFO_THROTTLE(10.0, "The Time of game %d", remain_time_);

    if (s_l_was_down_ && game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
    {
      bool is_opened = (dart_launch_opening_status_ == rm_msgs::DartClientCmd::OPENED);
      bool opening_for_prepare = is_gate_actually_opening_ &&
                                 dart_launch_opening_status_ == rm_msgs::DartClientCmd::OPENING_OR_CLOSING &&
                                 (ros::Time::now() - gate_opening_start_time_ > ros::Duration(3.5));
      bool gate_opened_timed_out = (is_opened && (ros::Time::now() - gate_opened_time_) > ros::Duration(31.0)) ||
                                   ((ros::Time::now() - gate_opening_start_time_) > ros::Duration(36.0));
      bool gate_available_for_start = (opening_for_prepare || is_opened) && !gate_opened_timed_out;
      bool enemy_outpost_alive = enemyOutpostAlive();
      bool enemy_outpost_blocks_fire = !enemy_outpost_ignore_ && enemy_outpost_alive;

      if (!enemy_outpost_ignore_ && !auto_shoot_active_ && !aborted_for_current_gate_ && gate_available_for_start &&
          launch_mode_ == INIT && enemy_outpost_alive)
      {
        ROS_INFO_THROTTLE(2.0, "Enemy outpost hp is %d. PC auto fire is waiting.", enemy_outpost_hp_);
      }

      if (!auto_shoot_active_ && !aborted_for_current_gate_ && gate_available_for_start && !enemy_outpost_blocks_fire &&
          launch_mode_ == INIT && ros::Time::now() - last_init_time_ > ros::Duration(0.4))
      {
        auto_shoot_active_ = true;
        auto_shoot_dart_count_ = 0;
        launch_mode_ = PULLDOWN;
        double elapsed = (ros::Time::now() - gate_opening_start_time_).toSec();

        ROS_INFO("Dart %d started charging. Time elapsed since gate start: %.2f s", auto_shoot_dart_count_ + 1, elapsed);

        if (pc_target_detection_)
        {
          target_found_once_ = false;
          camera_lost_start_time_ = ros::Time(0);
          pc_last_pulldown_time_ = ros::Time::now();
        }
      }

      if (auto_shoot_active_)
      {
        if (gate_opened_timed_out)
        {
          ROS_INFO("Gate opening timeout.");
          auto_shoot_active_ = false;
          aborted_for_current_gate_ = true;

          size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
          bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;
          if (recycle_mode_ && is_dart_active &&
              (launch_mode_ == ENGAGE || launch_mode_ == PULLUP || launch_mode_ == READY))
          {
            recycle_pulldown_start_time_ = ros::Time(0);
            recycle_pullup_start_time_ = ros::Time(0);
            recycle_start_time_ = ros::Time(0);
            dart_recycling_ = true;
            launch_mode_ = RECYCLE;
            readyLaunchDart(dart_fired_num_);
          }
          else
          {
            last_init_time_ = ros::Time::now();
            launch_mode_ = INIT;
          }

          return;
        }

        if (!enemy_outpost_ignore_ && enemy_outpost_alive && launch_mode_ != PUSH)
        {
          ROS_INFO_THROTTLE(1.0, "Enemy outpost revived, hp: %d. Recycle current dart and wait.", enemy_outpost_hp_);
          auto_shoot_active_ = false;
          target_found_once_ = false;
          camera_lost_start_time_ = ros::Time(0);

          size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
          bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;
          if (recycle_mode_ && is_dart_active &&
              (launch_mode_ == ENGAGE || launch_mode_ == PULLUP || launch_mode_ == READY))
          {
            recycle_pulldown_start_time_ = ros::Time(0);
            recycle_pullup_start_time_ = ros::Time(0);
            recycle_start_time_ = ros::Time(0);
            dart_recycling_ = true;
            launch_mode_ = RECYCLE;
            readyLaunchDart(dart_fired_num_);
          }
          else
          {
            last_init_time_ = ros::Time::now();
            launch_mode_ = INIT;
          }

          return;
        }

        if (pc_target_detection_)
        {
          bool should_abort = false;
          bool camera_pc_online = (ros::Time::now() - last_get_camera_data_time_ < ros::Duration(1.0));

          if (!camera_pc_online)
          {
            ROS_INFO_THROTTLE(1.0, "camera_pc_offline!!!");
            target_found_once_ = true;
            camera_lost_start_time_ = ros::Time::now();
            pc_last_pulldown_time_ = ros::Time::now();
          }
          else if (is_camera_found_)
          {
            target_found_once_ = true;
            camera_lost_start_time_ = ros::Time(0);
          }
          else
          {
            if (camera_lost_start_time_.isZero())
            {
              camera_lost_start_time_ = ros::Time::now();
            }
          }

          if (ros::Time::now() - pc_last_pulldown_time_ >= ros::Duration(5.0) && !target_found_once_)
          {
            should_abort = true;
            ROS_INFO_THROTTLE(2.0, "AutoFire Abort: Target not found within first 5s.");
          }
          else if (ros::Time::now() - pc_last_pulldown_time_ >= ros::Duration(5.0) &&
                   !camera_lost_start_time_.isZero() &&
                   (ros::Time::now() - camera_lost_start_time_) > ros::Duration(2.0))
          {
            should_abort = true;
            ROS_INFO_THROTTLE(2.0, "AutoFire Abort: Target lost for continuous 2s.");
          }

          if (should_abort)
          {
            auto_shoot_active_ = false;
            aborted_for_current_gate_ = true;

            size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
            bool is_dart_active =
                (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;

            if (recycle_mode_ && is_dart_active && (launch_mode_ == ENGAGE || launch_mode_ == PULLUP))
            {
              recycle_pulldown_start_time_ = ros::Time(0);
              recycle_pullup_start_time_ = ros::Time(0);
              recycle_start_time_ = ros::Time(0);
              dart_recycling_ = true;
              launch_mode_ = RECYCLE;
              readyLaunchDart(dart_fired_num_);
            }
            else
            {
              last_init_time_ = ros::Time::now();
              launch_mode_ = INIT;
            }
            return;
          }
        }

        if (auto_shoot_active_)
        {
          readyLaunchDart(dart_fired_num_);
        }

        if (auto_aim_start_)
        {
          autoAim();
        }

        switch (manual_state_)
        {
          case OUTPOST:
            yaw_sender_->setPoint(yaw_outpost_ + camera_x_set_point_);
            range_sender_->setPoint(range_outpost_ + camera_y_set_point_ + random_fixed_set_point_);
            break;
          case BASE:
            yaw_sender_->setPoint(yaw_base_ + camera_x_set_point_);
            range_sender_->setPoint(range_base_ + camera_y_set_point_ + random_fixed_set_point_);
            break;
          default:
            yaw_sender_->setPoint(yaw_base_ + camera_x_set_point_);
            range_sender_->setPoint(range_base_ + camera_y_set_point_ + random_fixed_set_point_);
            break;
        }

        if (launch_mode_ == READY && ros::Time::now() - vision_end_time_ > ros::Duration(0.3))
        {
          launch_mode_ = PUSH;
          last_push_time_ = ros::Time::now();
        }

        if (launch_mode_ == PUSH && triggerIsWorked() && (ros::Time::now() - last_push_time_ > ros::Duration(0.5)))
        {
          auto_shoot_dart_count_++;
          all_pc_fired_num_++;
          reset_belt_ = true;
          if (auto_shoot_dart_count_ >= pc_once_fire_num_)
          {
            auto_shoot_active_ = false;
            aborted_for_current_gate_ = true;
          }
          last_init_time_ = ros::Time::now();
          launch_mode_ = INIT;
        }
      }
    }
    else
    {
      auto_shoot_active_ = false;
    }
  }

  else
  {
    if (!pc_locked_)
    {
      ROS_INFO_THROTTLE(10.0, "referee_data_okay off, use vision to automatic.");
    }

    if (s_l_was_down_ && !pc_locked_)
    {
      if (is_camera_found_)
      {
        camera_target_lost_start_ = ros::Time(0);
        if (camera_target_visible_start_.isZero())
          camera_target_visible_start_ = ros::Time::now();

        if (!fake_gate_is_open_ && (ros::Time::now() - camera_target_visible_start_).toSec() > 1.0)
        {
          if (fake_gate_opened_time_.isZero() || (ros::Time::now() - fake_gate_opened_time_).toSec() > 40.0)
          {
            fake_gate_is_open_ = true;
            fake_gate_opened_time_ = ros::Time::now();
            aborted_for_current_gate_ = false;
            ROS_INFO("FAKE GATE: OPENED! Target tracked > 1s.");
          }
        }
      }
      else
      {
        camera_target_visible_start_ = ros::Time(0);
        if (camera_target_lost_start_.isZero())
          camera_target_lost_start_ = ros::Time::now();

        if (fake_gate_is_open_ && !camera_target_lost_start_.isZero() &&
            (ros::Time::now() - camera_target_lost_start_).toSec() > 1.0)
        {
          fake_gate_is_open_ = false;
          ROS_INFO("FAKE GATE: CLOSED! Target lost > 1s.");
        }
      }

      if (!auto_shoot_active_ && !aborted_for_current_gate_ && fake_gate_is_open_ && launch_mode_ == INIT &&
          ros::Time::now() - last_init_time_ > ros::Duration(0.5))
      {
        auto_shoot_active_ = true;
        auto_shoot_dart_count_ = 0;
        launch_mode_ = PULLDOWN;

        double elapsed = (ros::Time::now() - fake_gate_opened_time_).toSec();
        ROS_INFO("Dart %d started charging (Vision Mode). Time elapsed since target tracked: %.2f s",
                 auto_shoot_dart_count_ + 1, elapsed);
      }

      if (auto_shoot_active_)
      {
        if (fake_gate_is_open_ && !fake_gate_opened_time_.isZero() &&
            (ros::Time::now() - fake_gate_opened_time_).toSec() > 30.0)
        {
          ROS_INFO("Fake gate opening time > 30s. Closed fake gate.");
          auto_shoot_active_ = false;
          aborted_for_current_gate_ = true;
          last_init_time_ = ros::Time::now();
          launch_mode_ = INIT;
          return;
        }

        bool should_abort = !fake_gate_is_open_;

        if (should_abort)
        {
          auto_shoot_active_ = false;
          aborted_for_current_gate_ = true;

          size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
          bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;

          if (recycle_mode_ && is_dart_active &&
              (launch_mode_ == PULLDOWN || launch_mode_ == ENGAGE || launch_mode_ == PULLUP))
          {
            recycle_pulldown_start_time_ = ros::Time(0);
            recycle_pullup_start_time_ = ros::Time(0);
            recycle_start_time_ = ros::Time(0);
            dart_recycling_ = true;
            launch_mode_ = RECYCLE;
            readyLaunchDart(dart_fired_num_);
          }
          else
          {
            last_init_time_ = ros::Time::now();
            launch_mode_ = INIT;
          }
          return;
        }

        if (auto_shoot_active_)
        {
          readyLaunchDart(dart_fired_num_);
        }

        if (auto_aim_start_)
        {
          autoAim();
        }

        switch (manual_state_)
        {
          case OUTPOST:
            yaw_sender_->setPoint(yaw_outpost_ + camera_x_set_point_);
            range_sender_->setPoint(range_outpost_ + camera_y_set_point_ + random_fixed_set_point_);
            break;
          case BASE:
            yaw_sender_->setPoint(yaw_base_ + camera_x_set_point_);
            range_sender_->setPoint(range_base_ + camera_y_set_point_ + random_fixed_set_point_);
            break;
          default:
            yaw_sender_->setPoint(yaw_base_ + camera_x_set_point_);
            range_sender_->setPoint(range_base_ + camera_y_set_point_ + random_fixed_set_point_);
            break;
        }

        if (launch_mode_ == READY && ros::Time::now() - vision_end_time_ > ros::Duration(0.3))
        {
          launch_mode_ = PUSH;
          last_push_time_ = ros::Time::now();
        }

        if (launch_mode_ == PUSH && triggerIsWorked() && (ros::Time::now() - last_push_time_ > ros::Duration(0.4)))
        {
          auto_shoot_dart_count_++;
          all_pc_fired_num_++;
          reset_belt_ = true;
          if (auto_shoot_dart_count_ >= pc_once_fire_num_)
          {
            auto_shoot_active_ = false;
            aborted_for_current_gate_ = true;
          }
          last_init_time_ = ros::Time::now();
          launch_mode_ = INIT;
        }
      }
    }
    else
    {
      auto_shoot_active_ = false;
      fake_gate_is_open_ = false;
      camera_target_visible_start_ = ros::Time(0);
      camera_target_lost_start_ = ros::Time(0);
    }
  }

  if (!auto_shoot_active_ && dart_recycling_)
  {
    readyLaunchDart(dart_fired_num_);
  }
}

void Dart2Manual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  if (std::abs(dbus_data->ch_r_x) > std::abs(dbus_data->ch_r_y))
  {
    move(yaw_sender_, dbus_data->ch_r_x);
  }
  if (std::abs(dbus_data->ch_r_y) > std::abs(dbus_data->ch_r_x))
  {
    move(range_sender_, dbus_data->ch_r_y);
  }
  operateClamper(dbus_data);
}

void Dart2Manual::move(rm_common::JointPointCommandSender* joint, double ch)
{
  if (!joint_state_.position.empty())
  {
    int idx = joint->getIndex();
    if (idx == -1 || idx >= static_cast<int>(joint_state_.position.size()))
      return;
    double position = joint_state_.position[idx];
    if (ch != 0.)
    {
      joint->setPoint(position - ch * scale_);
      if_stop_ = true;
    }
    if (ch == 0. && if_stop_)
    {
      joint->setPoint(joint_state_.position[idx]);
      if_stop_ = false;
    }
  }
}

void Dart2Manual::updateLaunchMode()
{
  if (launch_mode_ != last_launch_mode_)
  {
    switch (launch_mode_)
    {
      case INIT:
        init();
        break;
      case PULLDOWN:
        pullDown();
        break;
      case ENGAGE:
        engage();
        break;
      case PULLUP:
        pullUp();
        break;
      case READY:
        ready();
        break;
      case PUSH:
        push();
        break;
      case RECYCLE:
        recycle();
        break;
      default:
        ROS_WARN("Invalid mode.");
        break;
    }
  }

  last_launch_mode_ = launch_mode_;
}

void Dart2Manual::init()
{
  ROS_INFO("Enter INIT");
  shooter_calibration_->reset();
  belt_left_sender_->setPoint(0.0);
  belt_right_sender_->setPoint(0.0);
  trigger_sender_->setPoint(trigger_home_command_);
  load_sender_->setPoint(load_init_position_);
  clamp_left_sender_->setPoint(0.0);
  clamp_mid_sender_->setPoint(0.0);
  clamp_right_sender_->setPoint(0.0);

  if (dart_fired_num_ > 3)
  {
    dart_fired_num_ = 0;
    ROS_INFO("Dart_fired_num reset to zero!");
  }
  last_init_time_ = ros::Time::now();
  auto_aim_state_ = NONE;
  random_fixed_set_point_ = 0.0;
  pulldown_start_time_ = ros::Time(0);
  pullup_start_time_ = ros::Time(0);

  if (dart_fired_num_ == 0)
  {
    camera_x_set_point_ = 0.0;
    current_x_offset_ = 0.0;
    camera_y_set_point_ = 0.0;
    current_y_offset_ = 0.0;
    camera_x_init_ = false;
  }

  start_aim_ = false;
  aim_failed_ = false;
  auto_aim_start_ = false;

  load_start_ = false;
  load_started_this_launch_ = false;
  load_finished_ = false;
  is_loading_ = false;
  belt_left_ready_ = false;
  belt_right_ready_ = false;
  dart_ready_ = false;
  vision_ready_ = false;
  retarget = false;
  clamp_finished_ = false;

  push_failed_ = false;
  push_succeeded_ = false;

  current_launch_auto_fire_ = false;
  current_launch_auto_fire_counted_ = false;

  dart_recycling_ = false;
  recycle_step_ = 0;
  recycle_pulldown_start_time_ = ros::Time(0);
  recycle_pullup_start_time_ = ros::Time(0);
  recycle_start_time_ = ros::Time(0);

  if (!camera_x_init_)
  {
    camera_x_after_push_ = 0;
    camera_x_init_ = true;
  }
}

void Dart2Manual::pullDown()
{
  ROS_INFO("Enter PULLDOWN");
  pulldown_start_time_ = ros::Time::now();
  trigger_sender_->setPoint(trigger_work_command_);
  belt_left_sender_->setPoint(belt_left_downward_vel_);
  belt_right_sender_->setPoint(belt_right_downward_vel_);
}

void Dart2Manual::engage()
{
  ROS_INFO("Enter ENGAGE");

  size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
  bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;

  if (is_dart_active)
  {
    trigger_sender_->setPoint(trigger_home_command_);
  }
  else
  {
    trigger_sender_->setPoint(trigger_work_command_);
    ROS_INFO("Dart %d is disabled", dart_fired_num_);
  }

  belt_left_sender_->setPoint(0.25);
  belt_right_sender_->setPoint(0.25);
  last_engage_time_ = ros::Time::now();

  if (camera_x_init_ && dart_fired_num_ != 0)
  {
    camera_x_after_push_ = camera_x_;
  }
}

void Dart2Manual::pullUp()
{
  ROS_INFO("Enter PULLUP");
  pullup_start_time_ = ros::Time::now();
  belt_left_sender_->setPoint(belt_right_upward_vel_);
  belt_right_sender_->setPoint(belt_left_upward_vel_);
}

void Dart2Manual::ready()
{
  ROS_INFO("Enter READY");
  belt_right_sender_->setPoint(0.0);
  belt_left_sender_->setPoint(0.0);
  trigger_sender_->setPoint(trigger_home_command_);
  last_ready_time_ = ros::Time::now();
  launch_result_ = false;
  load_skip_ = false;
}

void Dart2Manual::push()
{
  ROS_INFO("Enter PUSH");
  trigger_sender_->setPoint(trigger_work_command_);
  dart_fired_num_++;
  load_skip_ = false;
  load_started_this_launch_ = false;
  ROS_INFO("Launch dart num:%d", dart_fired_num_);
  last_push_time_ = ros::Time::now();
  load_finished_ = false;
}

void Dart2Manual::recycle()
{
  ROS_INFO("Enter RECYCLE");
  recycle_should_skip_load_ = load_started_this_launch_;
  recycle_step_ = 0;
  recycle_start_time_ = ros::Time::now();
  recycle_pulldown_start_time_ = ros::Time::now();
  recycle_pullup_start_time_ = ros::Time(0);
  belt_left_sender_->setPoint(6.0);
  belt_right_sender_->setPoint(6.0);
}

void Dart2Manual::readyLaunchDart(int dart_fired_num)
{
  if (is_loading_ && !load_skip_)
  {
    load_work();
    if (dart_ready_)
    {
      is_loading_ = false;
    }
  }

  switch (launch_mode_)
  {
    case INIT:
      if (isAutoFireRestrictionEnabled() && auto_fire_count_ >= auto_fire_num_)
      {
        ROS_INFO_THROTTLE(2.0, "Auto fire restriction reached: %d / %d. Blocking pulldown from INIT.", auto_fire_count_,
                          auto_fire_num_);
      }
      else if (shooter_calibration_->isCalibrated() && ros::Time::now() - last_init_time_ > ros::Duration(0.3) &&
               last_init_time_ > last_push_time_ && !dart_recycling_ &&
               ros::Time::now() - last_push_time_ > ros::Duration(0.6))
      {
        pulldown_start_time_ = ros::Time(0);
        launch_mode_ = PULLDOWN;
        last_init_time_ = ros::Time::now();

        if (state_ == PC && auto_shoot_active_)
        {
          double elapsed = referee_data_okay_ ? (ros::Time::now() - gate_opening_start_time_).toSec() :
                                                (ros::Time::now() - fake_gate_opened_time_).toSec();
          ROS_INFO("Dart %d started charging. Time elapsed since gate start: %.2f s", auto_shoot_dart_count_ + 1,
                   elapsed);
        }
      }
      break;

    case PULLDOWN:
      if ((belt_left_position_ >= belt_left_max_ && belt_right_position_ >= belt_right_max_ &&
           std::abs(load_velocity_) < 0.01) ||
          (pulldown_timeout_detect_ && !pulldown_start_time_.isZero() &&
           ros::Time::now() - pulldown_start_time_ > ros::Duration(pulldown_timeout_s_)))
      {
        if (pulldown_timeout_detect_ && !pulldown_start_time_.isZero() &&
            ros::Time::now() - pulldown_start_time_ > ros::Duration(pulldown_timeout_s_))
        {
          ROS_INFO("Pulldown timeout reached %.2f s, force entering ENGAGE.", pulldown_timeout_s_);
        }
        launch_mode_ = ENGAGE;
        last_engage_time_ = ros::Time::now();
      }
      break;

    case ENGAGE:
    {
      size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
      bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;

      if (is_dart_active)
      {
        if (triggerIsHome() && ros::Time::now() - last_engage_time_ > ros::Duration(0.5))
        {
          pullup_start_time_ = ros::Time(0);
          launch_mode_ = PULLUP;
          load_start_ = true;
          load_started_this_launch_ = true;
          is_loading_ = true;
          auto_aim_start_ = true;
        }
      }
      else
      {
        if (ros::Time::now() - last_engage_time_ > ros::Duration(false_engage_time_))
        {
          pullup_start_time_ = ros::Time(0);
          launch_mode_ = PULLUP;
          load_start_ = true;
          load_started_this_launch_ = true;
          is_loading_ = true;
          auto_aim_start_ = true;
        }
      }
      break;
    }

    case PULLUP:
      if (load_skip_)
      {
        ROS_INFO_THROTTLE(3.0, "Load skipped.");
        dart_ready_ = true;
        is_loading_ = false;
      }

      if (belt_left_position_ <= belt_left_slow_ && belt_left_position_ > belt_right_min_)
      {
        belt_left_sender_->setPoint(belt_left_upward_slow_vel_);
      }
      if (belt_right_position_ <= belt_right_slow_ && belt_right_position_ > belt_right_min_)
      {
        belt_right_sender_->setPoint(belt_right_upward_slow_vel_);
      }

      if (belt_left_position_ <= belt_left_min_)
      {
        belt_left_sender_->setPoint(0.0);
        belt_left_ready_ = true;
      }
      if (belt_right_position_ <= belt_right_min_)
      {
        belt_right_sender_->setPoint(0.0);
        belt_right_ready_ = true;
      }

      if (pullup_timeout_detect_ && !pullup_start_time_.isZero() &&
          ros::Time::now() - pullup_start_time_ > ros::Duration(pullup_timeout_s_))
      {
        if (!belt_left_ready_ || !belt_right_ready_)
        {
          ROS_INFO("Pullup timeout reached %.2f s, force entering READY.", pullup_timeout_s_);
        }
        belt_left_sender_->setPoint(0.0);
        belt_right_sender_->setPoint(0.0);
        belt_left_ready_ = true;
        belt_right_ready_ = true;
      }

      if (belt_left_ready_ && belt_right_ready_ && dart_ready_ && vision_ready_ && std::abs(range_velocity_) < 0.01)
      {
        launch_mode_ = READY;
        last_ready_time_ = ros::Time::now();
      }
      last_push_time_ = ros::Time::now();
      break;

    case PUSH:
      if (triggerIsWorked())
      {
        launch_result_ = true;
        ROS_INFO_THROTTLE(2.0, "Launch Dart successfully.");

        if (current_launch_auto_fire_ && !current_launch_auto_fire_counted_)
        {
          ++auto_fire_count_;
          current_launch_auto_fire_counted_ = true;
          ROS_INFO("Auto fire count: %d", auto_fire_count_);
        }

        if (auto_dart_ && ros::Time::now() - last_push_time_ > ros::Duration(1.5))
        {
          if (current_launch_auto_fire_ && isAutoFireRestrictionEnabled() && auto_fire_count_ >= auto_fire_num_)
          {
            ROS_INFO_THROTTLE(2.0, "Auto fire restriction reached: %d / %d. Auto dart recharge stopped.",
                              auto_fire_count_, auto_fire_num_);
          }
          else
          {
            launch_mode_ = INIT;
          }
        }
      }
      break;

    case RECYCLE:
      ROS_INFO_THROTTLE(1.0, "Now recycle_step_ is %d", recycle_step_);
      if (recycle_timeout_detect_ && !recycle_start_time_.isZero() &&
          ros::Time::now() - recycle_start_time_ > ros::Duration(20.0))
      {
        ROS_INFO("Recycle timeout reached 20s, force recycle finish.");
        belt_left_sender_->setPoint(0.0);
        belt_right_sender_->setPoint(0.0);
        left_recycle_done = true;
        right_recycle_done = true;
        dart_recycling_ = false;
        load_skip_ = recycle_should_skip_load_;
        launch_mode_ = INIT;
        break;
      }

      if (recycle_step_ == 0)
      {
        left_recycle_done = false;
        right_recycle_done = false;

        if ((belt_left_position_ >= belt_left_max_ && belt_right_position_ >= belt_right_max_) ||
            (recycle_pulldown_timeout_detect_ && !recycle_pulldown_start_time_.isZero() &&
             ros::Time::now() - recycle_pulldown_start_time_ > ros::Duration(10.0)))
        {
          if (recycle_pulldown_timeout_detect_ && !recycle_pulldown_start_time_.isZero() &&
              ros::Time::now() - recycle_pulldown_start_time_ > ros::Duration(10.0) &&
              !(belt_left_position_ >= belt_left_max_ && belt_right_position_ >= belt_right_max_))
          {
            ROS_INFO("Recycle pulldown timeout reached 10s, force entering step 1.");
          }
          recycle_step_ = 1;
          recycle_timer_ = ros::Time::now();
          ROS_INFO("Recycle: Reached bottom, waiting 1s...");
        }
      }
      else if (recycle_step_ == 1)
      {
        belt_left_sender_->setPoint(0.25);
        belt_right_sender_->setPoint(0.25);
        if (ros::Time::now() - recycle_timer_ > ros::Duration(1.0))
        {
          trigger_sender_->setPoint(trigger_work_command_);
          recycle_step_ = 2;
          recycle_timer_ = ros::Time::now();
          ROS_INFO("Recycle: Trigger released, waiting for trigger to open...");
        }
      }
      else if (recycle_step_ == 2)
      {
        if (ros::Time::now() - recycle_timer_ > ros::Duration(1.0) && triggerIsWorked())
        {
          belt_left_sender_->setPoint(belt_right_upward_vel_);
          belt_right_sender_->setPoint(belt_left_upward_vel_);
          recycle_step_ = 3;
          recycle_pullup_start_time_ = ros::Time::now();
          ROS_INFO("Recycle: Pulling up...");
        }
      }
      else if (recycle_step_ == 3)
      {
        if (belt_left_position_ <= belt_left_slow_ && belt_left_position_ > belt_right_min_)
        {
          belt_left_sender_->setPoint(belt_left_upward_slow_vel_);
        }
        if (belt_right_position_ <= belt_right_slow_ && belt_right_position_ > belt_right_min_)
        {
          belt_right_sender_->setPoint(belt_right_upward_slow_vel_);
        }

        if (belt_left_position_ <= belt_left_min_)
        {
          belt_left_sender_->setPoint(0.0);
          left_recycle_done = true;
        }
        else
        {
          left_recycle_done = false;
        }

        if (belt_right_position_ <= belt_right_min_)
        {
          belt_right_sender_->setPoint(0.0);
          right_recycle_done = true;
        }
        else
        {
          right_recycle_done = false;
        }

        if (pullup_timeout_detect_ && !recycle_pullup_start_time_.isZero() &&
            ros::Time::now() - recycle_pullup_start_time_ > ros::Duration(pullup_timeout_s_))
        {
          if (!left_recycle_done || !right_recycle_done)
          {
            ROS_INFO("Recycle pullup timeout reached %.2f s, force recycle finish.", pullup_timeout_s_);
          }
          belt_left_sender_->setPoint(0.0);
          belt_right_sender_->setPoint(0.0);
          left_recycle_done = true;
          right_recycle_done = true;
        }

        if (left_recycle_done && right_recycle_done)
        {
          dart_recycling_ = false;
          load_skip_ = recycle_should_skip_load_;
          launch_mode_ = INIT;
          ROS_INFO("Recycle finished. Ready for next launch without loading.");
        }
      }
      break;

    default:
      break;
  }
}

void Dart2Manual::leftSwitchDownOn()
{
  s_l_was_down_ = true;
  if (state_ != PC)
  {
    size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
    bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;

    if ((recycle_mode_ && is_dart_active &&
         (launch_mode_ == READY || launch_mode_ == PULLUP || launch_mode_ == ENGAGE)) ||
        dart_recycling_)
    {
      if (launch_mode_ != RECYCLE)
      {
        recycle_pulldown_start_time_ = ros::Time(0);
        recycle_pullup_start_time_ = ros::Time(0);
        recycle_start_time_ = ros::Time(0);
      }
      dart_recycling_ = true;
      launch_mode_ = RECYCLE;
      readyLaunchDart(dart_fired_num_);
    }
    else
    {
      launch_mode_ = INIT;
    }
  }
  else
  {
    ROS_INFO_THROTTLE(5.0, "PC mode ignore init.");
  }
}

void Dart2Manual::leftSwitchMidOn()
{
  s_l_was_down_ = false;
  switch (manual_state_)
  {
    case OUTPOST:
      yaw_sender_->setPoint(yaw_outpost_ + camera_x_set_point_);
      range_sender_->setPoint(range_outpost_ + camera_y_set_point_ + random_fixed_set_point_);
      break;
    case BASE:
      yaw_sender_->setPoint(yaw_base_ + camera_x_set_point_);
      range_sender_->setPoint(range_base_ + camera_y_set_point_ + random_fixed_set_point_);
      break;
    default:
      yaw_sender_->setPoint(yaw_base_ + camera_x_set_point_);
      range_sender_->setPoint(range_base_ + camera_y_set_point_ + random_fixed_set_point_);
      break;
  }
  readyLaunchDart(dart_fired_num_);
  if (auto_aim_start_)
  {
    autoAim();
  }

  if (auto_push_ && launch_mode_ == READY)
  {
    current_launch_auto_fire_ = (state_ != PC) && auto_dart_;
    current_launch_auto_fire_counted_ = false;
    launch_mode_ = PUSH;
  }
}

void Dart2Manual::leftSwitchUpOn()
{
  // if (launch_mode_ == READY) {
  if (launch_mode_ == READY && auto_aim_state_ == ADJUSTED && ros::Time::now() - vision_end_time_ > ros::Duration(0.5))
  {
    current_launch_auto_fire_ = false;
    current_launch_auto_fire_counted_ = false;
    last_push_time_ = ros::Time::now();
    launch_mode_ = PUSH;
  }

  // if (launch_mode_ == PUSH && ros::Time::now() - last_push_time_ > ros::Duration(0.5) && !triggerIsWorked() && !push_failed_
  // && !push_succeeded_) {
  //   trigger_sender_->setPoint(trigger_home_command_);
  //   push_failed_time = ros::Time::now();
  //   push_failed_ = true;
  // }
  // if (ros::Time::now() - push_failed_time > ros::Duration(2.5) && push_failed_ && !triggerIsWorked() && !push_succeeded_){
  //   trigger_sender_->setPoint(trigger_work_command_);
  //   last_push_time_ = ros::Time::now();
  //   push_failed_ = false;
  // }
  //
  // if (triggerIsWorked()) {
  //   push_succeeded_ = true;
  // }
}

void Dart2Manual::rightSwitchDownOn()
{
  recordPosition(dbus_data_);
  if (dbus_data_.ch_r_x >= -0.6)
  {
    load_init_toggle_active_ = false;
  }

  if (dbus_data_.ch_l_y == 1. && std::abs(dbus_data_.ch_r_x) < 0.3)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_sender_->setPoint(yaw_outpost_);
      range_sender_->setPoint(range_outpost_);
    }
    else if (manual_state_ == BASE)
    {
      yaw_sender_->setPoint(yaw_base_);
      range_sender_->setPoint(range_base_);
    }
  }
  if (dbus_data_.ch_l_y == -1. && std::abs(dbus_data_.ch_r_x) < 0.3)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_sender_->setPoint(target_position_["outpost"][0]);
      range_sender_->setPoint(target_position_["outpost"][1]);
    }
    else if (manual_state_ == BASE)
    {
      yaw_sender_->setPoint(target_position_["base"][0]);
      range_sender_->setPoint(target_position_["base"][1]);
    }
  }

  if (dbus_data_.ch_r_x > 0.6)
  {
    if (!test_auto_aim_active_)
    {
      int yaw_idx = yaw_sender_->getIndex();
      if (yaw_idx != -1 && yaw_idx < static_cast<int>(joint_state_.position.size()))
      {
        test_yaw_start_pos_ = joint_state_.position[yaw_idx];
        camera_x_set_point_ = test_yaw_start_pos_;
        test_auto_aim_active_ = true;
      }
    }

    if (test_auto_aim_active_)
    {
      if (is_camera_found_)
      {
        camera_x_set_point_ += camera_x_ * camera_slow_p_x_;
        yaw_sender_->setPoint(camera_x_set_point_);
      }
      else
      {
        yaw_sender_->setPoint(test_yaw_start_pos_);
        camera_x_set_point_ = test_yaw_start_pos_;
      }
    }
  }
  else if (dbus_data_.ch_r_x < -0.6)
  {
    // if (!load_init_toggle_active_)
    // {
    //   if (load_init_position_ < 0.03)
    //   {
    //     load_init_position_ = 0.043;
    //   }
    //   else if (load_init_position_ > 0.03)
    //   {
    //     load_init_position_ = 0.022;
    //   }
    //   load_sender_->setPoint(load_init_position_);
    //   load_init_toggle_active_ = true;
    //   ROS_INFO("Load init position switched to %lf", load_init_position_);
    // }

    camera_is_online_ = (ros::Time::now() - last_get_camera_data_time_ < ros::Duration(1.0));
    if (camera_is_online_)
    {
      trigger_sender_->setPoint(trigger_work_command_);
      test_camera_online_active_ = true;
    }
  }
  else
  {
    if (test_auto_aim_active_)
    {
      yaw_sender_->setPoint(test_yaw_start_pos_);
      test_auto_aim_active_ = false;
      camera_x_set_point_ = 0;
    }

    if (test_camera_online_active_)
    {
      trigger_sender_->setPoint(trigger_home_command_);
      test_camera_online_active_ = false;
    }
  }

  s_r_was_up_ = false;
}

void Dart2Manual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  allow_dart_door_open_times_ = 0;
  move_state_ = NORMAL;
  if (test_auto_aim_active_)
  {
    yaw_sender_->setPoint(test_yaw_start_pos_);
    test_auto_aim_active_ = false;
    camera_x_set_point_ = 0;
  }
  if (test_camera_online_active_)
  {
    trigger_sender_->setPoint(trigger_home_command_);
    test_camera_online_active_ = false;
  }

  all_pc_fired_num_ = 0;
  pc_locked_ = false;

  if (s_r_was_up_)
  {
    s_r_was_up_ = false;

    ROS_INFO("Manual Override: Right switch pulled from UP to MID. Aborting sequence.");
    auto_shoot_active_ = false;
    aborted_for_current_gate_ = true;
    target_found_once_ = false;
    has_entered_battle_ = false;

    size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
    bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;

    if (recycle_mode_ && is_dart_active && (launch_mode_ == ENGAGE || launch_mode_ == PULLUP || dart_recycling_))
    {
      dart_recycling_ = true;
      launch_mode_ = RECYCLE;
      readyLaunchDart(dart_fired_num_);
      ROS_INFO("Manual Override: Triggered RECYCLE recovery.");
    }
    else
    {
      launch_mode_ = INIT;
      last_init_time_ = ros::Time::now();
      ROS_INFO("Manual Override: Reset straight to INIT.");
    }
  }
}

void Dart2Manual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  if (test_auto_aim_active_)
  {
    yaw_sender_->setPoint(test_yaw_start_pos_);
    test_auto_aim_active_ = false;
    camera_x_set_point_ = 0;
  }
  if (test_camera_online_active_)
  {
    trigger_sender_->setPoint(trigger_home_command_);
    test_camera_online_active_ = false;
  }

  dart_fired_num_ = 0;
  all_pc_fired_num_ = 0;
  ROS_INFO("Dart_fired_num reset to zero.");

  s_r_was_up_ = true;
}

bool Dart2Manual::triggerIsHome() const
{
  return trigger_position_ <= trigger_confirm_home_;
}

bool Dart2Manual::triggerIsWorked() const
{
  return trigger_position_ >= trigger_confirm_work_;
}

void Dart2Manual::recordPosition(const rm_msgs::DbusData dbus_data)
{
  if (dbus_data.ch_r_y == 1.)
  {
    int yaw_idx = yaw_sender_->getIndex();
    int range_idx = range_sender_->getIndex();
    if (yaw_idx == -1 || range_idx == -1 || yaw_idx >= static_cast<int>(joint_state_.position.size()) ||
        range_idx >= static_cast<int>(joint_state_.position.size()))
      return;

    if (manual_state_ == OUTPOST)
    {
      yaw_outpost_ = joint_state_.position[yaw_idx];
      range_outpost_ = joint_state_.position[range_idx];
      ROS_INFO("Recorded outpost position.");
    }
    else if (manual_state_ == BASE)
    {
      yaw_base_ = joint_state_.position[yaw_idx];
      range_base_ = joint_state_.position[range_idx];
      ROS_INFO("Recorded base position.");
    }
  }
}

void Dart2Manual::sendCommand(const ros::Time& time)
{
  belt_left_sender_->sendCommand(time);
  belt_right_sender_->sendCommand(time);
  range_sender_->sendCommand(time);
  trigger_sender_->sendCommand(time);
  yaw_sender_->sendCommand(time);
  load_sender_->sendCommand(time);
  clamp_left_sender_->sendCommand(time);
  clamp_mid_sender_->sendCommand(time);
  clamp_right_sender_->sendCommand(time);
}

void Dart2Manual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  if (!joint_state_.name.empty() && !joint_state_.position.empty())
  {
    int belt_left_idx = belt_left_sender_->getIndex();
    int belt_right_idx = belt_right_sender_->getIndex();
    int trigger_idx = trigger_sender_->getIndex();
    int yaw_idx = yaw_sender_->getIndex();
    int range_idx = range_sender_->getIndex();
    int load_idx = load_sender_->getIndex();

    if (belt_left_idx != -1 && belt_left_idx < static_cast<int>(joint_state_.position.size()))
      belt_left_position_ = std::abs(joint_state_.position[belt_left_idx]);
    if (belt_right_idx != -1 && belt_right_idx < static_cast<int>(joint_state_.position.size()))
      belt_right_position_ = std::abs(joint_state_.position[belt_right_idx]);
    if (trigger_idx != -1 && trigger_idx < static_cast<int>(joint_state_.position.size()))
      trigger_position_ = joint_state_.position[trigger_idx];
    if (load_idx != -1 && load_idx < static_cast<int>(joint_state_.position.size()))
      load_position_ = joint_state_.position[load_idx];

    if (!joint_state_.velocity.empty())
    {
      if (yaw_idx != -1 && yaw_idx < static_cast<int>(joint_state_.velocity.size()))
        yaw_velocity_ = std::abs(joint_state_.velocity[yaw_idx]);
      if (range_idx != -1 && range_idx < static_cast<int>(joint_state_.velocity.size()))
        range_velocity_ = std::abs(joint_state_.velocity[range_idx]);
      if (load_idx != -1 && load_idx < static_cast<int>(joint_state_.velocity.size()))
        load_velocity_ = std::abs(joint_state_.velocity[load_idx]);
    }
  }
  wheel_clockwise_event_.update(data->wheel == 1.0);
  wheel_anticlockwise_event_.update(data->wheel == -1.0);
  dbus_data_ = *data;
}

void Dart2Manual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  game_progress_ = data->game_progress;

  if (game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
    remain_time_ = data->stage_remain_time;
  else
    remain_time_ = 421;
}

void Dart2Manual::dartInfoCallback(const rm_msgs::DartInfo::ConstPtr& data)
{
  dart_remaining_time_ = data->dart_remaining_time;
  dart_current_target_ = data->dart_current_target;
}

void Dart2Manual::gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
{
  enemy_outpost_hp_ = data->enemy_outpost_hp;
  if (enemy_outpost_hp_ <= 200)
  {
    enemy_outpost_dangerous_ = true;
  }
  else
  {
    enemy_outpost_dangerous_ = false;
  }
}

void Dart2Manual::dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data)
{
  if (dart_launch_opening_status_ != data->dart_launch_opening_status)
  {
    ROS_INFO("Dart gate status changed: %d -> %d", dart_launch_opening_status_, data->dart_launch_opening_status);
  }

  if (dart_launch_opening_status_ != rm_msgs::DartClientCmd::OPENING_OR_CLOSING &&
      data->dart_launch_opening_status == rm_msgs::DartClientCmd::OPENING_OR_CLOSING)
  {
    if (dart_launch_opening_status_ == rm_msgs::DartClientCmd::CLOSED || dart_launch_opening_status_ == 3)
    {
      gate_opening_start_time_ = ros::Time::now();
      aborted_for_current_gate_ = false;
      is_gate_actually_opening_ = true;
    }
  }

  if (dart_launch_opening_status_ != rm_msgs::DartClientCmd::OPENED &&
      data->dart_launch_opening_status == rm_msgs::DartClientCmd::OPENED)
  {
    gate_opened_time_ = ros::Time::now();
    is_gate_actually_opening_ = false;
  }

  if (data->dart_launch_opening_status == rm_msgs::DartClientCmd::CLOSED)
  {
    is_gate_actually_opening_ = false;
  }

  dart_launch_opening_status_ = data->dart_launch_opening_status;
  target_change_time_ = data->target_change_time;
}

void Dart2Manual::wheelClockwise()
{
  switch (clamp_num_)
  {
    case 0:
      clamp_num_ = 1;
      temp_load_position_ = load_init_position_;
      break;
    case 1:
      clamp_num_ = 2;
      temp_load_position_ = load_left_position_;
      break;
    case 2:
      clamp_num_ = 3;
      temp_load_position_ = load_mid_position_;
      break;
    case 3:
      temp_load_position_ = load_right_position_;
      clamp_num_ = 0;
      break;
  }
  ROS_INFO("Now temp_load_position_: %lf", temp_load_position_);

  // -------------------------------------

  // switch (move_state_)
  // {
  //   case NORMAL:
  //     scale_ = scale_micro_;
  //     move_state_ = MICRO;
  //     ROS_INFO("Pitch and yaw : MICRO_MOVE_MODE");
  //     break;
  //   case MICRO:
  //     scale_ = 0.01;
  //     move_state_ = NORMAL;
  //     ROS_INFO("Pitch and yaw : NORMAL_MOVE_MODE");
  //     break;
  // }

  // -------------------------------------

  // dart_fired_num_ = dart_fired_num_ + 1;

  // load_skip_ = false;

  // ROS_INFO("Changed launch dart num:%d", dart_fired_num_);
}

void Dart2Manual::wheelAntiClockwise()
{
  // if (temp_load_)
  // {
  //   load_sender_->setPoint(0.04);
  //   temp_load_ = false;
  // }
  // else {
  //   load_sender_->setPoint(0.04);
  //   temp_load_ = true;
  // }

  // -------------------------------------

  load_sender_->setPoint(temp_load_position_);

  // -------------------------------------

  // if (dart_fired_num_ != 0) {
  //   dart_fired_num_ = dart_fired_num_ - 1;
  // }
  //
  // load_skip_ = false;
  //
  // ROS_INFO("Changed launch dart num:%d", dart_fired_num_);
}

/*
void Dart2Manual::longCameraDataCallback(const rm_msgs::Dart::ConstPtr& data)
{
  is_long_camera_found_ = data->is_found;
  long_camera_x_ = data->distance;
  long_camera_y_ = data->height;
  last_get_camera_data_time_ = data->stamp;
}
*/

void Dart2Manual::cameraDataCallback(const rm_msgs::Dart::ConstPtr& data)
{
  is_camera_found_ = data->is_found;
  camera_x_ = data->distance;
  camera_y_ = data->height;
  last_get_camera_data_time_ = data->stamp;
}

void Dart2Manual::updateAutoAimState()
{
  switch (auto_aim_state_)
  {
    case AIM:
      aim();
      break;
    case ADJUST:
      adjust();
      break;
  }
  last_auto_aim_state_ = auto_aim_state_;
}

void Dart2Manual::aim()
{
  if (last_auto_aim_state_ != auto_aim_state_)
  {
    ROS_INFO("ENTER AIM.");
    start_aim_ = true;
    start_aim_time_ = ros::Time::now();
    random_fixed_set_point_ = 0.0;
  }
  if (is_camera_found_ && std::abs(camera_x_) >= camera_fast_x_threshold_)
  {
    camera_x_set_point_ += camera_x_ * camera_fast_p_x_;
  }
  if (is_camera_found_ && std::abs(camera_x_) < camera_fast_x_threshold_ &&
      std::abs(camera_x_) >= camera_normal_x_threshold_)
  {
    camera_x_set_point_ += camera_x_ * camera_normal_p_x_;
  }
  if (is_camera_found_ && std::abs(camera_x_) < camera_normal_x_threshold_ &&
      std::abs(camera_x_) >= camera_slow_x_threshold_)
  {
    if (retarget)
    {
      camera_x_set_point_ += camera_x_ * camera_retarget_slow_p_x_;
    }
    else
    {
      camera_x_set_point_ += camera_x_ * camera_slow_p_x_;
    }
  }

  if (auto_aim_state_ == AIM && std::abs(camera_x_) <= camera_slow_x_threshold_ && is_camera_found_ &&
      yaw_velocity_ < 0.01)
  {
    int yaw_idx = yaw_sender_->getIndex();
    double yaw_pos = (yaw_idx != -1 && yaw_idx < static_cast<int>(joint_state_.position.size())) ?
                         joint_state_.position[yaw_idx] :
                         0.0;
    ROS_INFO("The %d dart had aimed.Now x error:%lf, yaw position: %lf ", dart_fired_num_, camera_x_, yaw_pos);
    if (random_fixed_target_ && random_fixed_compensation_)
    {
      random_fixed_set_point_ =
          (yaw_pos - random_fixed_reference_yaw_position_) * random_fixed_compensation_coefficient_;
      ROS_INFO("Random fixed range compensation: yaw_delta: %lf, range_set_point: %lf",
               yaw_pos - random_fixed_reference_yaw_position_, random_fixed_set_point_);
    }
    else
    {
      random_fixed_set_point_ = 0.0;
    }
    ROS_INFO("aim time used: %lf s", (ros::Time::now() - start_aim_time_).toSec());
    current_x_offset_ = 0.0;
    start_aim_ = false;
    auto_aim_state_ = AIMED;
  }
}

void Dart2Manual::adjust()
{
  if (last_auto_aim_state_ != auto_aim_state_)
  {
    ROS_INFO("ENTER ADJUST.");
    last_adjust_time_ = ros::Time::now();
  }

  if (!had_adjust_)
  {
    target_x_offset = dart_list_[dart_fired_num_].base_offset_;
    target_y_offset_ = dart_list_[dart_fired_num_].base_range_;
    ROS_INFO("Adjusting Offset. Removing old: %lf, Adding new: %lf", current_x_offset_, target_x_offset);

    camera_x_set_point_ -= current_x_offset_;
    camera_x_set_point_ += target_x_offset;
    current_x_offset_ = target_x_offset;

    camera_y_set_point_ -= current_y_offset_;
    camera_y_set_point_ += target_y_offset_;
    current_y_offset_ = target_y_offset_;

    had_adjust_ = true;
  }

  if (yaw_velocity_ < 0.001 && ros::Time::now() - last_adjust_time_ > ros::Duration(0.1) && range_velocity_ < 0.001)
  {
    int yaw_idx = yaw_sender_->getIndex();
    int range_idx = range_sender_->getIndex();
    double yaw_pos = (yaw_idx != -1 && yaw_idx < static_cast<int>(joint_state_.position.size())) ?
                         joint_state_.position[yaw_idx] :
                         0.0;
    double range_pos = (range_idx != -1 && range_idx < static_cast<int>(joint_state_.position.size())) ?
                           joint_state_.position[range_idx] :
                           0.0;
    ROS_INFO("The %d dart had adjusted.Now x error:%lf, yaw position: %lf ,range position: %lf", dart_fired_num_,
             camera_x_, yaw_pos, range_pos);
    camera_x_before_push_ = camera_x_;
    had_adjust_ = false;
    auto_aim_start_ = false;
    auto_aim_state_ = ADJUSTED;
    vision_ready_ = true;
    vision_end_time_ = ros::Time::now();
  }
}

void Dart2Manual::autoAim()
{
  camera_is_online_ = (ros::Time::now() - last_get_camera_data_time_ < ros::Duration(1.0));

  if (use_auto_aim_ && camera_is_online_)
  {
    if (start_aim_ && ros::Time::now() - start_aim_time_ > ros::Duration(6.0))
    {
      start_aim_ = false;
      random_fixed_set_point_ = 0.0;
      ROS_INFO("Auto aim time out for 6s!!!");
      aim_failed_ = true;
      if (keep_auto_aim_effect_on_timeout_)
      {
        ROS_INFO("Keeping auto aim effect at timeout. x_set_point: %lf, y_set_point: %lf", camera_x_set_point_,
                 camera_y_set_point_);
      }
      else
      {
        camera_x_set_point_ = current_x_offset_;
        camera_y_set_point_ = current_y_offset_;
      }

      auto_aim_state_ = ADJUST;
    }

    if (!aim_failed_)
    {
      if (random_fixed_target_)
      {
        ROS_INFO_THROTTLE(5.0, "random_fixed_target is enabled, auto aim will be forced for every dart.");
      }
      const bool should_auto_aim_current_dart = shouldAutoAimCurrentDart();
      if (should_auto_aim_current_dart)
      {
        if (auto_aim_state_ != ADJUSTED && auto_aim_state_ != ADJUST && auto_aim_state_ != AIMED &&
            auto_aim_state_ != AIM)
        {
          auto_aim_state_ = AIM;
        }
      }
      else
      {
        if (auto_aim_state_ != ADJUSTED && auto_aim_state_ != ADJUST && auto_aim_state_ != AIMED &&
            auto_aim_state_ != AIM)
        {
          if (std::abs(camera_x_after_push_ - camera_x_before_push_) < retarget_threshold_)
          {
            auto_aim_state_ = ADJUST;
          }
          else
          {
            retarget = true;
            auto_aim_state_ = AIM;
          }
        }
      }
    }

    if (auto_aim_state_ == AIMED)
    {
      auto_aim_state_ = ADJUST;
    }
  }
  else if (auto_aim_state_ != ADJUST && auto_aim_state_ != ADJUSTED)
  {
    ROS_INFO("use_auto_aim False or Camera offline, aim disabled.");
    camera_x_set_point_ = current_x_offset_;
    camera_y_set_point_ = current_y_offset_;
    random_fixed_set_point_ = 0.0;

    auto_aim_state_ = ADJUST;
  }
}

bool Dart2Manual::shouldAutoAimCurrentDart() const
{
  if (!use_auto_aim_)
  {
    return false;
  }

  if (random_fixed_target_)
  {
    return true;
  }

  if (custom_auto_aim_)
  {
    const size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
    return current_dart_idx < auto_aim_status_.size() ? auto_aim_status_[current_dart_idx] : false;
  }

  return dart_fired_num_ == 0;
}

bool Dart2Manual::isAutoFireRestrictionEnabled() const
{
  return state_ != PC && auto_push_ && auto_dart_ && auto_fire_restriction_;
}

bool Dart2Manual::enemyOutpostAlive() const
{
  if (enemy_outpost_danger_detect_)
  {
    return !enemy_outpost_dangerous_;
  }

  return enemy_outpost_hp_ > 0;
}

void Dart2Manual::load_dart(int step_idx)
{
  switch (step_idx)
  {
    case 0:
      current_load_target_ = load_init_position_;
      break;
    case 1:
      current_load_target_ = load_left_position_;
      break;
    case 2:
      clamp_left_sender_->setPoint(clamp_left_release_position_);
      current_load_target_ = load_mid_position_;
      break;
    case 3:
      clamp_left_sender_->setPoint(clamp_left_release_position_);
      clamp_mid_sender_->setPoint(clamp_mid_release_position_);
      current_load_target_ = load_right_position_;
      break;
    default:
      return;
  }

  load_sender_->setPoint(current_load_target_);
  ROS_INFO_THROTTLE(1.0, "Load Executing Step %d: Target Pos %.3f", step_idx, current_load_target_);
}

void Dart2Manual::load_work()
{
  if (use_load_)
  {
    int load_idx = load_sender_->getIndex();
    if (load_idx == -1 || load_idx >= static_cast<int>(joint_state_.position.size()))
      return;
    cur_pos = joint_state_.position[load_idx];
    // load_position_ = cur_pos;

    if (!clamp_finished_ && !joint_state_.position.empty())
    {
      load_dart(dart_fired_num_ % 4);
    }

    if (dart_fired_num_ % 4 == 0 && std::abs(load_velocity_) < 0.001 && !joint_state_.position.empty())
    {
      load_start_ = false;
      clamp_finished_ = true;
      load_finished_ = true;
      dart_ready_ = true;
      load_sender_->setPoint(load_init_position_);
    }

    if (!joint_state_.position.empty())
    {
      if (std::abs(cur_pos - current_load_target_) < 0.03 && std::abs(load_velocity_) < 0.001 && !load_finished_)
      {
        load_finish_time_ = ros::Time::now();
        load_finished_ = true;
        load_start_ = false;
      }
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "joint_state_.position IS EMPTY!!");
      load_start_ = false;
      load_finished_ = true;
      clamp_finished_ = true;
      dart_ready_ = true;
    }

    if (!clamp_finished_)
    {
      last_clamp_finished_time_ = ros::Time::now();
    }

    if (!clamp_finished_ && load_finished_ && ros::Time::now() - load_finish_time_ > ros::Duration(0.6))
    {
      switch (dart_fired_num_ % 4)
      {
        case 1:
          clamp_left_sender_->setPoint(clamp_left_release_position_);
          break;
        case 2:
          clamp_mid_sender_->setPoint(clamp_mid_release_position_);
          break;
        case 3:
          clamp_right_sender_->setPoint(clamp_right_release_position_);
          break;
        case 0:
          break;
      }
      clamp_finished_ = true;
    }

    if (clamp_finished_ && load_finished_ && (ros::Time::now() - last_clamp_finished_time_) > ros::Duration(0.4))
    {
      load_sender_->setPoint(load_init_position_);
      if (std::abs(cur_pos - load_init_position_) < 0.04)
      {
        dart_ready_ = true;
      }
    }
  }
  else
  {
    ROS_INFO_THROTTLE(2.0, "use_load is false, skip load process.");
    dart_ready_ = true;
  }
}

void Dart2Manual::operateClamper(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  if (dbus_data->ch_l_y > 0.75 && clamp_manual_)
  {
    clamp_left_sender_->setPoint(0.0);
    clamp_mid_sender_->setPoint(0.0);
    clamp_right_sender_->setPoint(0.0);
    return;
  }
  if (dbus_data->ch_l_x > 0.75 && clamp_manual_)
  {
    clamp_right_sender_->setPoint(clamp_right_release_position_);
    return;
  }
  if (dbus_data->ch_l_y < -0.75 && clamp_manual_)
  {
    clamp_mid_sender_->setPoint(clamp_mid_release_position_);
    return;
  }
  if (dbus_data->ch_l_x < -0.75 && clamp_manual_)
  {
    clamp_left_sender_->setPoint(clamp_left_release_position_);
    return;
  }
}

}  // namespace rm_manual
