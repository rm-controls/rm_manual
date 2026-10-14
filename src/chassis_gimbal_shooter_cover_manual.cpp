//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"

namespace rm_manual
{
ChassisGimbalShooterCoverManual::ChassisGimbalShooterCoverManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalShooterManual(nh, nh_referee)
{
  if (nh.hasParam("base_pitch"))
  {
    ros::NodeHandle base_pitch_nh(nh, "base_pitch");
    std::string base_pitch_topic{};
    base_pitch_nh.param("topic", base_pitch_topic, std::string("/controllers/base_pitch_controller/command"));
    base_pitch_pub_ = base_pitch_nh.advertise<std_msgs::Float64>(base_pitch_topic, 1);
    zipped_pitch_rate_pid_ = std::make_shared<control_toolbox::Pid>();
    if (base_pitch_nh.hasParam("zipped_pitch_rate_pid"))
    {
      zipped_pitch_rate_pid_->init(ros::NodeHandle(base_pitch_nh, "zipped_pitch_rate_pid"));
    }
  }

  ros::NodeHandle wireless_nh(nh, "wireless");
  nh.param("wireless_frame", wireless_frame_, std::string("wireless_frame"));
  ros::NodeHandle buff_switch_nh(nh, "buff_switch");
  switch_buff_srv_ = new rm_common::SwitchDetectionCaller(buff_switch_nh);
  ros::NodeHandle buff_type_switch_nh(nh, "buff_type_switch");
  switch_buff_type_srv_ = new rm_common::SwitchDetectionCaller(buff_type_switch_nh);
  ros::NodeHandle exposure_switch_nh(nh, "exposure_switch");
  switch_exposure_srv_ = new rm_common::SwitchDetectionCaller(exposure_switch_nh, "/hk_camera/exposure_status_switch");
  ros::NodeHandle chassis_nh(nh, "chassis");
  normal_speed_scale_ = chassis_nh.param("normal_speed_scale", 1);
  low_speed_scale_ = chassis_nh.param("low_speed_scale", 0.30);
  nh.param("exit_buff_mode_duration", exit_buff_mode_duration_, 0.5);
  nh.param("gyro_speed_limit", gyro_speed_limit_, 6.0);
  ros::NodeHandle vel_nh(nh, "vel");
  sin_gyro_base_scale_ = vel_nh.param("sin_gyro_base_scale", 1.0);
  sin_gyro_amplitude_ = vel_nh.param("sin_gyro_amplitude", 0.0);
  sin_gyro_period_ = vel_nh.param("sin_gyro_period", 1.0);
  sin_gyro_phase_ = vel_nh.param("sin_gyro_phase", 0.0);

  ctrl_z_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this),
                        boost::bind(&ChassisGimbalShooterCoverManual::ctrlZRelease, this));
  ctrl_x_event_.setRising(boost::bind(&ChassisGimbalShooterCoverManual::ctrlXPress, this));
  ctrl_r_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterCoverManual::ctrlRPressing, this));
  r_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::rPress, this),
                   boost::bind(&ChassisGimbalShooterCoverManual::rRelease, this));
  e_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::ePress, this),
                   boost::bind(&ChassisGimbalShooterCoverManual::eRelease, this));
  q_event_.setRising(boost::bind(&ChassisGimbalShooterCoverManual::qPress, this));
  z_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::zPress, this),
                   boost::bind(&ChassisGimbalShooterCoverManual::zRelease, this));
}

void ChassisGimbalShooterCoverManual::remoteControlTurnOn()
{
  ChassisGimbalShooterManual::remoteControlTurnOn();
  zipped_ = true;
}

void ChassisGimbalShooterCoverManual::changeSpeedMode(SpeedMode speed_mode)
{
  if (speed_mode == LOW)
  {
    speed_change_scale_ = low_speed_scale_;
  }
  else if (speed_mode == NORMAL)
  {
    speed_change_scale_ = normal_speed_scale_;
  }
}

double ChassisGimbalShooterCoverManual::getDynamicScale(const double base_scale, const double amplitude,
                                                        const double period, const double phase)
{
  ros::Time current_time = ros::Time::now();
  double t = current_time.toSec();
  double f = 2 * M_PI / period;
  double dynamic_scale = base_scale + amplitude * sin(f * t + phase);
  if (dynamic_scale < 0.0)
  {
    dynamic_scale = 0.0;
  }
  else if (dynamic_scale > 1.0)
  {
    dynamic_scale = 1.0;
  }
  return dynamic_scale;
}

void ChassisGimbalShooterCoverManual::changeGyroSpeedMode(SpeedMode speed_mode)
{
  if (speed_mode == LOW)
  {
    if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_, gyro_speed_limit_);
    else
      vel_cmd_sender_->setAngularZVel(1.0, gyro_speed_limit_);
  }
  else if (speed_mode == NORMAL)
  {
    if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(1.0);
  }
}

void ChassisGimbalShooterCoverManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterManual::updatePc(dbus_data);
  if (ros::Time::now().toSec() - 1.0 > gimbal_output_on_time_.toSec())
  {
    gimbal_cmd_sender_->setRate(-dbus_data->m_x * gimbal_scale_, -dbus_data->m_y * gimbal_scale_);
  }
  else
  {
    double pitch_err = 0.0;
    getPitchErr(pitch_err);
    double cmd_rate = zipped_pitch_rate_pid_->computeCommand(pitch_err, ros::Duration(0.01));
    gimbal_cmd_sender_->getMsg()->rate_pitch = cmd_rate;
    gimbal_cmd_sender_->getMsg()->rate_yaw = 0.0;
  }
  if (is_gyro_)
  {
    if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
      if (x_scale_ != 0.0 || y_scale_ != 0.0)
        vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_, gyro_speed_limit_);
      else
        vel_cmd_sender_->setAngularZVel(1.0, gyro_speed_limit_);
    else if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(
          getDynamicScale(sin_gyro_base_scale_, sin_gyro_amplitude_, sin_gyro_period_, sin_gyro_phase_) *
          gyro_rotate_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(
          getDynamicScale(sin_gyro_base_scale_, sin_gyro_amplitude_, sin_gyro_period_, sin_gyro_phase_));
  }
}

void ChassisGimbalShooterCoverManual::checkReferee()
{
  if (switch_detection_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    manual_to_referee_pub_data_.det_target = switch_buff_type_srv_->getTarget();
  else
    manual_to_referee_pub_data_.det_target = switch_detection_srv_->getTarget();
  manual_to_referee_pub_data_.zip_state = zipped_;
  ChassisGimbalShooterManual::checkReferee();
}

void ChassisGimbalShooterCoverManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterManual::checkKeyboard(dbus_data);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);
}

void ChassisGimbalShooterCoverManual::getPitchErr(double& err)
{
  static int pitch_index = -1;
  static int base_pitch_index = -1;
  if (pitch_index < 0)
  {
    auto it = std::find(joint_state_.name.begin(), joint_state_.name.end(), "pitch_joint");
    if (it == joint_state_.name.end())
    {
      ROS_WARN_THROTTLE(1.0, "Pitch joint %s not found in joint_states.", "pitch_joint");
      return;
    }
    pitch_index = std::distance(joint_state_.name.begin(), it);
  }
  if (base_pitch_index < 0)
  {
    auto it = std::find(joint_state_.name.begin(), joint_state_.name.end(), "base_pitch_joint");
    if (it == joint_state_.name.end())
    {
      ROS_WARN_THROTTLE(1.0, "Base pitch joint %s not found in joint_states.", "base_pitch_joint");
      return;
    }
    base_pitch_index = std::distance(joint_state_.name.begin(), it);
  }
  if (pitch_index >= 0 && base_pitch_index >= 0 && static_cast<size_t>(pitch_index) < joint_state_.position.size() &&
      static_cast<size_t>(base_pitch_index) < joint_state_.position.size())
  {
    err = -joint_state_.position[base_pitch_index] - joint_state_.position[pitch_index];
  }
}

void ChassisGimbalShooterCoverManual::sendCommand(const ros::Time& time)
{
  if (need_wireless_)
  {
    chassis_cmd_sender_->getMsg()->follow_source_frame = wireless_frame_;
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  else
    chassis_cmd_sender_->getMsg()->follow_source_frame = "yaw";

  if (base_pitch_pub_)
  {
    std_msgs::Float64 cmd;
    if (zipped_)
    {
      cmd.data = 0.0;
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
      double pitch_err = 0.0;
      getPitchErr(pitch_err);
      double cmd_rate = zipped_pitch_rate_pid_->computeCommand(pitch_err, ros::Duration(0.01));
      gimbal_cmd_sender_->getMsg()->rate_pitch = cmd_rate;
    }
    else
    {
      cmd.data = 0.5;
    }
    base_pitch_pub_.publish(cmd);
  }
  ChassisGimbalShooterManual::sendCommand(time);
}

void ChassisGimbalShooterCoverManual::rightSwitchDownRise()
{
  ChassisGimbalShooterManual::rightSwitchDownRise();
  zipped_ = true;
}

void ChassisGimbalShooterCoverManual::rightSwitchMidRise()
{
  ChassisGimbalShooterManual::rightSwitchMidRise();
  zipped_ = true;
}

void ChassisGimbalShooterCoverManual::rightSwitchUpRise()
{
  ChassisGimbalShooterManual::rightSwitchUpRise();
  zipped_ = false;
}

void ChassisGimbalShooterCoverManual::leftSwitchMidRise()
{
  ChassisGimbalShooterManual::leftSwitchMidRise();
  zipped_ = false;
}

void ChassisGimbalShooterCoverManual::leftSwitchDownRise()
{
  ChassisGimbalShooterManual::leftSwitchDownRise();
  zipped_ = true;
}

void ChassisGimbalShooterCoverManual::mouseRightPress()
{
  ChassisGimbalShooterManual::mouseRightPress();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR &&
      gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRACK)
  {
    if (shooter_cmd_sender_->getMsg()->mode == rm_msgs::ShootCmd::STOP)
    {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
      prepare_shoot_ = true;
    }
  }
}

void ChassisGimbalShooterCoverManual::ePress()
{
  switch_buff_srv_->setTargetType(rm_msgs::StatusChangeRequest::SMALL_BUFF);
  switch_detection_srv_->setTargetType(rm_msgs::StatusChangeRequest::SMALL_BUFF);
  switch_buff_type_srv_->setTargetType(rm_msgs::StatusChangeRequest::SMALL_BUFF);
  switch_exposure_srv_->setTargetType(rm_msgs::StatusChangeRequest::SMALL_BUFF);
  switch_buff_srv_->callService();
  switch_detection_srv_->callService();
  switch_buff_type_srv_->callService();
  switch_exposure_srv_->callService();
  if (is_gyro_)
    changeGyroSpeedMode(LOW);
  last_shoot_freq_ = shooter_cmd_sender_->getShootFrequency();
  shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::MINIMAL);
}

void ChassisGimbalShooterCoverManual::eRelease()
{
  ChassisGimbalShooterManual::eRelease();
  switch_buff_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
  switch_detection_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
  switch_buff_type_srv_->setTargetType(switch_buff_srv_->getTarget());
  switch_exposure_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
  switch_buff_srv_->callService();
  switch_detection_srv_->callService();
  switch_buff_type_srv_->callService();
  switch_exposure_srv_->callService();
  shooter_cmd_sender_->setShootFrequency(last_shoot_freq_);
  if (is_gyro_)
    changeGyroSpeedMode(NORMAL);
}

void ChassisGimbalShooterCoverManual::bPress()
{
}

void ChassisGimbalShooterCoverManual::cPress()
{
  setChassisMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    changeGyroSpeedMode(LOW);
  else
    changeGyroSpeedMode(NORMAL);
}

void ChassisGimbalShooterCoverManual::qPress()
{
  setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
}

void ChassisGimbalShooterCoverManual::wPress()
{
  ChassisGimbalShooterManual::wPress();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    last_switch_time_ = ros::Time::now();
}

void ChassisGimbalShooterCoverManual::wPressing()
{
  ChassisGimbalShooterManual::wPressing();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::aPressing()
{
  ChassisGimbalShooterManual::aPressing();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::sPressing()
{
  ChassisGimbalShooterManual::sPressing();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::dPressing()
{
  ChassisGimbalShooterManual::dPressing();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::wRelease()
{
  ChassisGimbalShooterManual::wRelease();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::aRelease()
{
  ChassisGimbalShooterManual::aRelease();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::sRelease()
{
  ChassisGimbalShooterManual::sRelease();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::dRelease()
{
  ChassisGimbalShooterManual::dRelease();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::zPress()
{
  switch_buff_srv_->setTargetType(rm_msgs::StatusChangeRequest::BIG_BUFF);
  switch_detection_srv_->setTargetType(rm_msgs::StatusChangeRequest::BIG_BUFF);
  switch_buff_type_srv_->setTargetType(rm_msgs::StatusChangeRequest::BIG_BUFF);
  switch_exposure_srv_->setTargetType(rm_msgs::StatusChangeRequest::BIG_BUFF);
  switch_buff_srv_->callService();
  switch_detection_srv_->callService();
  switch_buff_type_srv_->callService();
  switch_exposure_srv_->callService();
  if (is_gyro_)
    changeGyroSpeedMode(LOW);
  last_shoot_freq_ = shooter_cmd_sender_->getShootFrequency();
  shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::MINIMAL);
}

void ChassisGimbalShooterCoverManual::zRelease()
{
  ChassisGimbalShooterManual::eRelease();
  switch_buff_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
  switch_detection_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
  switch_buff_type_srv_->setTargetType(switch_buff_srv_->getTarget());
  switch_exposure_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
  switch_buff_srv_->callService();
  switch_detection_srv_->callService();
  switch_buff_type_srv_->callService();
  switch_exposure_srv_->callService();
  shooter_cmd_sender_->setShootFrequency(last_shoot_freq_);
  if (is_gyro_)
    changeGyroSpeedMode(NORMAL);
}

void ChassisGimbalShooterCoverManual::rPress()
{
  zipped_ = true;
}

void ChassisGimbalShooterCoverManual::rRelease()
{
  zipped_ = false;
}

void ChassisGimbalShooterCoverManual::ctrlZPress()
{
}

void ChassisGimbalShooterCoverManual::ctrlZRelease()
{
}

void ChassisGimbalShooterCoverManual::ctrlXPress()
{
  if (!need_wireless_)
  {
    need_wireless_ = true;
    chassis_cmd_sender_->setWirelessState(true);
  }
  else
  {
    need_wireless_ = false;
    chassis_cmd_sender_->setWirelessState(false);
  }
}

void ChassisGimbalShooterCoverManual::ctrlRPressing()
{
  if (!is_gyro_)
  {
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    setChassisMode(rm_msgs::ChassisCmd::RAW);
  }
  if (track_data_.id == 0)
  {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
    double traj_yaw = M_PI * count_ / 1000;
    double traj_pitch = 0.0;
    count_++;
    gimbal_cmd_sender_->setGimbalTraj(traj_yaw, traj_pitch);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
  else
  {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(ros::Time::now());
  }
}

void ChassisGimbalShooterCoverManual::ctrlRRelease()
{
  count_ = 0;
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterCoverManual::shiftPress()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void ChassisGimbalShooterCoverManual::gimbalOutputOn()
{
  gimbal_output_on_time_ = ros::Time::now();
  ChassisGimbalShooterManual::gimbalOutputOn();
}

}  // namespace rm_manual
