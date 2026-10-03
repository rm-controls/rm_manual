//
// Created by longyu on 9/21/25.
//

#include "rm_manual/series_legged_manual.h"

namespace rm_manual
{
SeriesLeggedManual::SeriesLeggedManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : BalanceManual(nh, nh_referee)
{
  ros::NodeHandle leg_wheel_chassis_nh(nh, "balance/legged_wheel_chassis");
  legCommandSender_ = new rm_common::LegCommandSender(leg_wheel_chassis_nh);

  double short_leg_len{}, mid_leg_length{}, high_leg_len{}, retraction_leg_len{};
  leg_wheel_chassis_nh.param("short_leg_length", short_leg_len, 0.2);
  leg_wheel_chassis_nh.param("mid_leg_length", mid_leg_length, 0.26);
  leg_wheel_chassis_nh.param("high_leg_length", high_leg_len, 0.36);
  leg_wheel_chassis_nh.param("retraction_leg_length", retraction_leg_len, 0.12);
  nh.param("debug_gimbal_flag", debug_gimbal_flag_, false);
  leg_len_map_.emplace(SHORT, short_leg_len);
  leg_len_map_.emplace(MID, mid_leg_length);
  leg_len_map_.emplace(HIGH, high_leg_len);
  leg_len_map_.emplace(LEG_RETRACTION, retraction_leg_len);
  legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
  legCommandSender_->setJump(false);

  b_event_.setEdge(boost::bind(&SeriesLeggedManual::bPress, this), boost::bind(&SeriesLeggedManual::bRelease, this));
  b_event_.setActiveHigh(boost::bind(&SeriesLeggedManual::bPressing, this));
  r_event_.setEdge(boost::bind(&SeriesLeggedManual::rPress, this), boost::bind(&SeriesLeggedManual::rRelease, this));
  r_event_.setActiveHigh(boost::bind(&SeriesLeggedManual::rPressing, this));
  f_event_.setEdge(boost::bind(&SeriesLeggedManual::fPress, this), boost::bind(&SeriesLeggedManual::fRelease, this));
  v_event_.setFalling(boost::bind(&SeriesLeggedManual::vRelease, this));
  ctrl_g_event_.setRising(boost::bind(&SeriesLeggedManual::ctrlGPress, this));
  ctrl_g_event_.setFalling(boost::bind(&SeriesLeggedManual::ctrlGRelease, this));
  ctrl_w_event_.setActiveHigh(boost::bind(&SeriesLeggedManual::ctrlWPressing, this));
  revive_motor_online_check_event_.setRising(boost::bind(&SeriesLeggedManual::exitSitDown, this));
  revive_motor_online_check_event_.setFalling(boost::bind(&SeriesLeggedManual::enterSitDown, this));
  std::string unstick_topic, upstair_status_topic, legged_chassis_mode_topic, tof_topic;
  leg_wheel_chassis_nh.param("unstick_topic", unstick_topic,
                             std::string("/controllers/legged_balance_controller/unstick/two_leg_unstick"));
  leg_wheel_chassis_nh.param("upstair_status_topic", upstair_status_topic,
                             std::string("/controllers/chassis_controller/upstair_status"));
  leg_wheel_chassis_nh.param("legged_chassis_mode_topic", legged_chassis_mode_topic,
                             std::string("/controllers/chassis_controller/legged_chassis_mode"));
  leg_wheel_chassis_nh.param("tof_topic", tof_topic, std::string("/tof"));
  leg_wheel_chassis_nh.param("jump_up_range_threshold", jump_up_range_threshold_, 0.9);
  unstick_sub_ =
      leg_wheel_chassis_nh.subscribe<std_msgs::Bool>(unstick_topic, 1, &SeriesLeggedManual::unstickCallback, this);
  leg_len_status_sub_ = leg_wheel_chassis_nh.subscribe<rm_msgs::LeggedUpstairStatus>(
      upstair_status_topic, 1, &SeriesLeggedManual::upstairStatusCallback, this);
  legged_chassis_mode_sub_ = leg_wheel_chassis_nh.subscribe<rm_msgs::LeggedChassisMode>(
      legged_chassis_mode_topic, 10, &SeriesLeggedManual::leggedChassisModeCallback, this);
  left_tof_sensor_sub_ = leg_wheel_chassis_nh.subscribe<sensor_msgs::Range>(
      tof_topic + "/left_tof_link", 10, &SeriesLeggedManual::tofSensorMsgCallback, this);
  right_tof_sensor_sub_ = leg_wheel_chassis_nh.subscribe<sensor_msgs::Range>(
      tof_topic + "/right_tof_link", 10, &SeriesLeggedManual::tofSensorMsgCallback, this);
  revive_motor_online_sub_ = leg_wheel_chassis_nh.subscribe<rm_ecat_msgs::RmEcatStandardSlaveReadings>(
      "/rm_ecat_hw/rm_readings", 10, &SeriesLeggedManual::reviveMotorOnlineCallback, this);
  recovery_leg_spd_turnback_pub_ = leg_wheel_chassis_nh.advertise<std_msgs::Bool>("/recovery_leg_spd_turnback", 1);
  down_5cm_stair_client_ = leg_wheel_chassis_nh.serviceClient<std_srvs::Trigger>("/down_5cm_stair");
  xPress_time_ = ctrlXPress_time_ = ros::Time::now();
}

void SeriesLeggedManual::sendCommand(const ros::Time& time)
{
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
  {
    double roll{}, pitch{}, yaw_normal{}, yaw_reverse{};
    try
    {
      quatToRPY(tf_buffer_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation, roll, pitch,
                yaw_normal);
      quatToRPY(tf_buffer_.lookupTransform("base_link", reverse_frame_, ros::Time(0)).transform.rotation, roll, pitch,
                yaw_reverse);

      if ((time - xPress_time_).toSec() > 3.0f && (time - ctrlXPress_time_).toSec() > 3.0f)
      {
        if (std::abs(yaw_reverse) < std::abs(yaw_normal))
        {
          if (!reverse_)
          {
            reverse_ = true;
          }
        }
        else
        {
          if (reverse_)
          {
            reverse_ = false;
          }
        }
      }
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_ONCE("%s", ex.what());
    }
  }
  BalanceManual::sendCommand(time);
  if (jump_up_flag_)
  {
    if (total_tof_len_ < jump_up_range_threshold_)
    {
      legCommandSender_->setJump(true);
      jump_up_flag_ = false;
      setLegLenStatus(SHORT);
    }
  }
  else
  {
    legCommandSender_->setJump(false);
  }
  legCommandSender_->sendCommand(time);
}

void SeriesLeggedManual::xPress()
{
  xPress_time_ = ros::Time::now();
  BalanceManual::xPress();
}

void SeriesLeggedManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterCoverManual::checkKeyboard(dbus_data);
  ctrl_g_event_.update(dbus_data->key_ctrl && dbus_data->key_g);
  ctrl_event_.update(dbus_data->key_ctrl);
  ctrl_w_event_.update(dbus_data->key_ctrl && dbus_data->key_w && !dbus_data->key_g);
  f_event_.update(dbus_data->key_f && !dbus_data->key_ctrl);
}

void SeriesLeggedManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  BalanceManual::updateRc(dbus_data);
  if (!is_gyro_)
  {  // Capacitor enter fast charge when chassis stop.
    if (!dbus_data->wheel && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW &&
        std::sqrt(std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) >
            0.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    else if (chassis_power_ < 6.0 && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
  else
  {
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
  if (is_die_flag_ || debug_gimbal_flag_)
  {
    setChassisMode(rm_msgs::ChassisCmd::FALLEN);
  }
}

void SeriesLeggedManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  BalanceManual::updatePc(dbus_data);
  if (is_die_flag_ || debug_gimbal_flag_)
  {
    setChassisMode(rm_msgs::ChassisCmd::FALLEN);
  }
}

void SeriesLeggedManual::rightSwitchDownRise()
{
  BalanceManual::rightSwitchDownRise();
}

void SeriesLeggedManual::rightSwitchMidRise()
{
  BalanceManual::rightSwitchMidRise();
}

void SeriesLeggedManual::ctrlZPress()
{
  std_msgs::Bool msg;
  msg.data = true;
  recovery_leg_spd_turnback_pub_.publish(msg);
}

void SeriesLeggedManual::shiftRelease()
{
  //  BalanceManual::shiftRelease();
  setLegLenStatus(SHORT);
}

void SeriesLeggedManual::shiftPress()
{
  setLegLenStatus(MID);
}

void SeriesLeggedManual::rPress()
{
  if (!stretch_)
  {
    upstair_leg_len_fsm_ = 1;
    setLegLenStatus(HIGH);
    legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
    stretch_ = true;
  }
  else
  {
    upstair_leg_len_fsm_ = 0;
    setLegLenStatus(SHORT);
    legCommandSender_->setLgeLength(leg_len_map_[leg_len_status_]);
    stretch_ = false;
  }
}

void SeriesLeggedManual::rPressing()
{
}

void SeriesLeggedManual::rRelease()
{
  legCommandSender_->setJump(false);
}

void SeriesLeggedManual::bPress()
{
  jump_up_flag_ = true;
  setLegLenStatus(LEG_RETRACTION);
}

void SeriesLeggedManual::bRelease()
{
  jump_up_flag_ = true;
  setLegLenStatus(LEG_RETRACTION);
}

void SeriesLeggedManual::ctrlGPress()
{
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RECOVERY);
}

void SeriesLeggedManual::ctrlGRelease()
{
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
}

void SeriesLeggedManual::ctrlWPressing()
{
  if ((ros::Time::now() - last_upstairs_time_) > ros::Duration(3.0))
  {
    if (total_tof_len_ < jump_up_range_threshold_)
    {
      last_upstairs_time_ = ros::Time::now();
      leg_len_status_ = HIGH;
    }
    else
    {
      leg_len_status_ = SHORT;
    }
    setLegLenStatus(leg_len_status_);
  }
}

void SeriesLeggedManual::unstickCallback(const std_msgs::BoolConstPtr& msg)
{
  auto two_leg_unstick = msg->data;
  if (two_leg_unstick)
  {
    auto delta = legCommandSender_->getLgeLength() + 0.02;
    target_leg_length_ = delta > 0.36 ? 0.36 : delta;
    legCommandSender_->setLgeLength(target_leg_length_);
    stretching_ = true;
  }
  else if (stretching_ && !two_leg_unstick)
  {
    legCommandSender_->setLgeLength(target_leg_length_);
    stretching_ = false;
  }
}

void SeriesLeggedManual::upstairStatusCallback(const rm_msgs::LeggedUpstairStatusConstPtr& msg)
{
  if (msg->upstair_flag)
    --upstair_leg_len_fsm_;
  if (upstair_leg_len_fsm_ == 1)
    leg_len_status_ = HIGH;
  else if (upstair_leg_len_fsm_ <= 0)
  {
    leg_len_status_ = SHORT;
    upstair_leg_len_fsm_ = 0;
  }
  setLegLenStatus(leg_len_status_);
}

void SeriesLeggedManual::leggedChassisModeCallback(const rm_msgs::LeggedChassisModeConstPtr& msg)
{
  static bool trigger_gimbal_normal_flag{ false }, gimbal_controller_open_flag{ true };
  static bool trigger_gimbal_traj_flag = true;
  if (msg->mode != rm_msgs::LeggedChassisMode::NORMAL && !debug_gimbal_flag_)
  {
    double roll{}, pitch{}, yaw{};
    try
    {
      quatToRPY(tf_buffer_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation, roll, pitch, yaw);
    }
    catch (tf2::TransformException& ex)
    {
      //      ROS_WARN("%s", ex.what());
    }
    gimbal_cmd_sender_->setGimbalTrajFrameId("base_link");
    if (msg->mode == rm_msgs::LeggedChassisMode::RECOVERY)
    {
      controller_manager_.stopController("gimbal_controller");
      gimbal_controller_open_flag = false;
      reverse_ = false;
    }
    else
    {
      if (!gimbal_controller_open_flag)
      {
        gimbal_controller_open_flag = true;
        controller_manager_.startController("gimbal_controller");
      }
      else
      {
        static ros::Time last_gimbal_traj_time = ros::Time::now();
        if (trigger_gimbal_traj_flag)
        {
          gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
          // avoid leg crash gimbal
          gimbal_cmd_sender_->setGimbalTraj(-0.0, pitch);
          last_gimbal_traj_time = ros::Time::now();
          trigger_gimbal_traj_flag = false;
        }
        else if ((ros::Time::now().toSec() - last_gimbal_traj_time.toSec()) > 1.5f)
        {
          gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        }
      }
    }
    trigger_gimbal_normal_flag = true;
    speed_change_scale_ = leg_len_status_ == HIGH ? 0.7 : 1.0;
  }
  else
  {
    trigger_gimbal_traj_flag = true;
    if (trigger_gimbal_normal_flag)
    {
      trigger_gimbal_normal_flag = false;
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    }
  }
  if (down_5cm_stair_flag_)
  {
    down_5cm_stair_flag_ = false;
    setLegLenStatus(SHORT);
    speed_change_scale_ = 1.0;
  }
}

void SeriesLeggedManual::tofSensorMsgCallback(const sensor_msgs::RangeConstPtr& msg)
{
  if (msg->header.frame_id == "left_tof_link")
    left_tof_len_ = msg->range;
  else
    right_tof_len = msg->range;
  total_tof_len_ = (left_tof_len_ + right_tof_len) / 2.0;
}

void SeriesLeggedManual::leftSwitchUpRise()
{
  BalanceManual::leftSwitchUpRise();
  //  legCommandSender_->setJump(false);
  //  upstair_leg_len_fsm_ = 1;
  //  setLegLenStatus(HIGH);

  // <unused>
  //  legCommandSender_->setJump(true);
  return;
}

void SeriesLeggedManual::leftSwitchMidRise()
{
  BalanceManual::leftSwitchMidRise();
  //  legCommandSender_->setJump(false);
  //  upstair_leg_len_fsm_ = 0;
  //  setLegLenStatus(MID);

  //  legCommandSender_->setJump(false);
  //  upstair_leg_len_fsm_ = 0;
  //  setLegLenStatus(SHORT);

  // <unused>
  //  legCommandSender_->setJump(true);
}

void SeriesLeggedManual::leftSwitchDownRise()
{
  BalanceManual::leftSwitchDownRise();
  legCommandSender_->setJump(false);
  upstair_leg_len_fsm_ = 0;
  setLegLenStatus(SHORT);

  //  legCommandSender_->setJump(false);
  //  upstair_leg_len_fsm_ = 0;
  //  setLegLenStatus(MID);
}
void SeriesLeggedManual::zPress()
{
  //  reverse_ = !reverse_;
  if (sit_down_flag_)
  {
    sit_down_flag_ = false;
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
  else
  {
    sit_down_flag_ = true;
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  }
}

void SeriesLeggedManual::ctrlXPress()
{
  ctrlXPress_time_ = ros::Time::now();
  reverse_ = !reverse_;
}

void SeriesLeggedManual::robotDie()
{
  setChassisMode(rm_msgs::ChassisCmd::FALLEN);
  is_die_flag_ = true;
  ChassisGimbalShooterManual::robotDie();
}

void SeriesLeggedManual::robotRevive()
{
  rm_manual::ChassisGimbalShooterManual::ManualBase::robotRevive();
  setChassisMode(rm_msgs::ChassisCmd::FALLEN);
}

void SeriesLeggedManual::exitSitDown()
{
  is_die_flag_ = false;
  setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  ROS_INFO("exit sit down mode for motor offline");
}

void SeriesLeggedManual::enterSitDown()
{
  is_die_flag_ = true;
  setChassisMode(rm_msgs::ChassisCmd::FALLEN);
  ROS_INFO("Enter sit down mode for motor offline");
}

void SeriesLeggedManual::reviveMotorOnlineCallback(const rm_ecat_msgs::RmEcatStandardSlaveReadingsConstPtr& msg)
{
  for (const auto& reading : msg->readings)
  {
    for (uint32_t i = 0; i < reading.names.size(); ++i)
    {
      if (reading.names[i] == "left_wheel_joint_motor")
      {
        left_wheel_online_ = reading.isOnline[i];
      }
      else if (reading.names[i] == "right_wheel_joint_motor")
      {
        right_wheel_online_ = reading.isOnline[i];
      }
    }
  }
  revive_motor_online_check_event_.update(left_wheel_online_ && right_wheel_online_);
}

inline void SeriesLeggedManual::setLegLenStatus(Leg_len_status len_status)
{
  leg_len_status_ = len_status;
  target_leg_length_ = leg_len_map_[leg_len_status_];
  legCommandSender_->setLgeLength(target_leg_length_);
  speed_change_scale_ = leg_len_status_ == HIGH ? 0.7 : 1.0;
}
void SeriesLeggedManual::rightSwitchDownOn()
{
  ManualBase::rightSwitchDownOn();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
}

void SeriesLeggedManual::aPress()
{
}

void SeriesLeggedManual::aPressing()
{
}

void SeriesLeggedManual::aRelease()
{
}

void SeriesLeggedManual::dPress()
{
}

void SeriesLeggedManual::dPressing()
{
}

void SeriesLeggedManual::dRelease()
{
}

void SeriesLeggedManual::ePress()
{
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FALLEN);
  ChassisGimbalShooterCoverManual::ePress();
}

void SeriesLeggedManual::eRelease()
{
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  ChassisGimbalShooterCoverManual::eRelease();
}

void SeriesLeggedManual::qPress()
{
  ChassisGimbalShooterCoverManual::qPress();
  down_5cm_stair_flag_ = jump_up_flag_ = false;
  legCommandSender_->setJump(false);
  setLegLenStatus(SHORT);
}

void SeriesLeggedManual::bPressing()
{
  jump_up_flag_ = true;
  setLegLenStatus(LEG_RETRACTION);
}
void SeriesLeggedManual::fRelease()
{
}
void SeriesLeggedManual::fPress()
{
  shooter_cmd_sender_->raiseSpeed();
}
void SeriesLeggedManual::vPress()
{
  std_srvs::Trigger srv;
  down_5cm_stair_client_.call(srv);
  speed_change_scale_ = 0.2f;
  down_5cm_stair_flag_ = true;
}
void SeriesLeggedManual::vRelease()
{
}
}  // namespace rm_manual
