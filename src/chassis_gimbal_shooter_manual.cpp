//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_manual.h"

namespace rm_manual
{
ChassisGimbalShooterManual::ChassisGimbalShooterManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalManual(nh, nh_referee)
{
  ros::NodeHandle shooter_nh(nh, "shooter");
  shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh);
  if (nh.hasParam("camera"))
  {
    ros::NodeHandle camera_nh(nh, "camera");
    camera_switch_cmd_sender_ = new rm_common::CameraSwitchCommandSender(camera_nh);
  }
  if (nh.hasParam("scope"))
  {
    ros::NodeHandle scope_nh(nh, "scope");
    scope_cmd_sender_ = new rm_common::JointPositionBinaryCommandSender(scope_nh);
  }
  if (nh.hasParam("image_transmission"))
  {
    ros::NodeHandle image_transmission_nh(nh, "image_transmission");
    image_transmission_cmd_sender_ = new rm_common::JointPositionBinaryCommandSender(image_transmission_nh);
  }

  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);
  ros::NodeHandle armor_target_switch_nh(nh, "armor_target_switch");
  switch_armor_target_srv_ = new rm_common::SwitchDetectionCaller(armor_target_switch_nh);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("shooter_calibration", rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  shooter_power_on_event_.setRising(boost::bind(&ChassisGimbalShooterManual::shooterOutputOn, this));
  self_inspection_event_.setRising(boost::bind(&ChassisGimbalShooterManual::selfInspectionStart, this));
  game_start_event_.setRising(boost::bind(&ChassisGimbalShooterManual::gameStart, this));
  left_switch_up_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::leftSwitchUpOn, this, _1));
  left_switch_up_event_.setFalling(boost::bind(&ChassisGimbalShooterManual::leftSwitchUpFall, this));
  left_switch_mid_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::leftSwitchMidOn, this, _1));
  e_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::ePress, this),
                   boost::bind(&ChassisGimbalShooterManual::eRelease, this));
  c_event_.setRising(boost::bind(&ChassisGimbalShooterManual::cPress, this));
  q_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::qPress, this),
                   boost::bind(&ChassisGimbalShooterManual::qRelease, this));
  b_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::bPress, this),
                   boost::bind(&ChassisGimbalShooterManual::bRelease, this));
  x_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::xPress, this),
                   boost::bind(&ChassisGimbalShooterManual::xRelease, this));
  r_event_.setRising(boost::bind(&ChassisGimbalShooterManual::rPress, this));
  g_event_.setRising(boost::bind(&ChassisGimbalShooterManual::gPress, this));
  v_event_.setRising(boost::bind(&ChassisGimbalShooterManual::vPress, this));
  ctrl_f_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlFPress, this));
  ctrl_v_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlVPress, this));
  ctrl_b_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlBPress, this));
  ctrl_q_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlQPress, this));
  ctrl_r_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::ctrlRPress, this),
                        boost::bind(&ChassisGimbalShooterManual::ctrlRRelease, this));
  shift_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::shiftPress, this),
                       boost::bind(&ChassisGimbalShooterManual::shiftRelease, this));
  mouse_left_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::mouseLeftPress, this));
  mouse_left_event_.setFalling(boost::bind(&ChassisGimbalShooterManual::mouseLeftRelease, this));
  mouse_right_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::mouseRightPress, this));
  mouse_right_event_.setFalling(boost::bind(&ChassisGimbalShooterManual::mouseRightRelease, this));
}

void ChassisGimbalShooterManual::run()
{
  ChassisGimbalManual::run();
  shooter_calibration_->update(ros::Time::now());
  gimbal_calibration_->update(ros::Time::now());
}

void ChassisGimbalShooterManual::checkReferee()
{
  manual_to_referee_pub_data_.power_limit_state = chassis_cmd_sender_->power_limit_->getState();
  manual_to_referee_pub_data_.shoot_frequency = shooter_cmd_sender_->getShootFrequency();
  manual_to_referee_pub_data_.gimbal_eject = gimbal_cmd_sender_->getEject();
  manual_to_referee_pub_data_.det_armor_target = switch_armor_target_srv_->getArmorTarget();
  manual_to_referee_pub_data_.det_color = switch_detection_srv_->getColor();
  manual_to_referee_pub_data_.det_exposure = switch_detection_srv_->getExposureLevel();
  manual_to_referee_pub_data_.stamp = ros::Time::now();
  ChassisGimbalManual::checkReferee();
}

void ChassisGimbalShooterManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::checkKeyboard(dbus_data);
  e_event_.update(dbus_data->key_e & !dbus_data->key_ctrl);
  c_event_.update((!dbus_data->key_ctrl) & dbus_data->key_c);
  g_event_.update(dbus_data->key_g & !dbus_data->key_ctrl);
  q_event_.update((!dbus_data->key_ctrl) & dbus_data->key_q);
  b_event_.update((!dbus_data->key_ctrl && !dbus_data->key_shift) & dbus_data->key_b);
  x_event_.update(dbus_data->key_x & !dbus_data->key_ctrl);
  r_event_.update((!dbus_data->key_ctrl) & dbus_data->key_r);
  v_event_.update((!dbus_data->key_ctrl) & dbus_data->key_v);
  ctrl_f_event_.update(dbus_data->key_f & dbus_data->key_ctrl);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
  ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b & !dbus_data->key_shift);
  ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);
  ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
  shift_event_.update(dbus_data->key_shift & !dbus_data->key_ctrl);
  ctrl_shift_b_event_.update(dbus_data->key_ctrl & dbus_data->key_shift & dbus_data->key_b);
  mouse_left_event_.update(dbus_data->p_l & !dbus_data->key_ctrl);
  mouse_right_event_.update(dbus_data->p_r & !dbus_data->key_ctrl);
}

void ChassisGimbalShooterManual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ChassisGimbalManual::gameRobotStatusCallback(data);
  shooter_cmd_sender_->updateGameRobotStatus(*data);
}

void ChassisGimbalShooterManual::powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
{
  ChassisGimbalManual::powerHeatDataCallback(data);
  shooter_cmd_sender_->updatePowerHeatData(*data);
}

void ChassisGimbalShooterManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ChassisGimbalManual::dbusDataCallback(data);
  chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
  shooter_cmd_sender_->updateRefereeStatus(referee_is_online_);
}

void ChassisGimbalShooterManual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ChassisGimbalManual::gameStatusCallback(data);
  self_inspection_event_.update(data->game_progress == 2);
  game_start_event_.update(data->game_progress == 4);
}

void ChassisGimbalShooterManual::gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data)
{
  ChassisGimbalManual::gimbalDesErrorCallback(data);
  shooter_cmd_sender_->updateGimbalDesError(*data);
}

void ChassisGimbalShooterManual::shootBeforehandCmdCallback(const rm_msgs::ShootBeforehandCmd ::ConstPtr& data)
{
  ChassisGimbalManual::shootBeforehandCmdCallback(data);
  shooter_cmd_sender_->updateShootBeforehandCmd(*data);
}

void ChassisGimbalShooterManual::trackCallback(const rm_msgs::TrackData::ConstPtr& data)
{
  ChassisGimbalManual::trackCallback(data);
  shooter_cmd_sender_->updateTrackData(*data);
  vel_cmd_sender_->updateTrackData(*data);
}

void ChassisGimbalShooterManual::suggestFireCallback(const std_msgs::Bool::ConstPtr& data)
{
  ManualBase::suggestFireCallback(data);
  shooter_cmd_sender_->updateSuggestFireData(*data);
}

void ChassisGimbalShooterManual::shootDataCallback(const rm_msgs::ShootData::ConstPtr& data)
{
  ChassisGimbalManual::shootDataCallback(data);
  if (referee_is_online_ && !use_scope_)
    shooter_cmd_sender_->updateShootData(*data);
}

void ChassisGimbalShooterManual::sendCommand(const ros::Time& time)
{
  ChassisGimbalManual::sendCommand(time);
  shooter_cmd_sender_->sendCommand(time);
  if (camera_switch_cmd_sender_)
    camera_switch_cmd_sender_->sendCommand(time);
  if (scope_cmd_sender_)
  {
    if (!use_scope_)
      scope_cmd_sender_->off();
    else
      scope_cmd_sender_->on();
    scope_cmd_sender_->sendCommand(time);
  }
  if (image_transmission_cmd_sender_)
  {
    if (!adjust_image_transmission_)
      image_transmission_cmd_sender_->off();
    else
      image_transmission_cmd_sender_->on();
    image_transmission_cmd_sender_->sendCommand(time);
  }
}

void ChassisGimbalShooterManual::remoteControlTurnOff()
{
  ChassisGimbalManual::remoteControlTurnOff();
  shooter_cmd_sender_->setZero();
  shooter_calibration_->stop();
  gimbal_calibration_->stop();
  use_scope_ = false;
  adjust_image_transmission_ = false;
}

void ChassisGimbalShooterManual::remoteControlTurnOn()
{
  ChassisGimbalManual::remoteControlTurnOn();
  shooter_calibration_->stopController();
  gimbal_calibration_->stopController();
  std::string robot_color = robot_id_ >= 100 ? "blue" : "red";
  switch_detection_srv_->setEnemyColor(robot_id_, robot_color);
}

void ChassisGimbalShooterManual::robotDie()
{
  ManualBase::robotDie();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  use_scope_ = false;
  adjust_image_transmission_ = false;
}

void ChassisGimbalShooterManual::chassisOutputOn()
{
  ChassisGimbalManual::chassisOutputOn();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void ChassisGimbalShooterManual::shooterOutputOn()
{
  ChassisGimbalManual::shooterOutputOn();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  shooter_calibration_->reset();
}

void ChassisGimbalShooterManual::gimbalOutputOn()
{
  ChassisGimbalManual::gimbalOutputOn();
  gimbal_calibration_->reset();
}

void ChassisGimbalShooterManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updateRc(dbus_data);
  if (std::abs(dbus_data->wheel) > 0.01)
  {
    setChassisMode(rm_msgs::ChassisCmd::RAW);
  }
  else
  {
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  vel_cmd_sender_->setAngularZVel((std::abs(dbus_data->ch_r_y) > 0.01 || std::abs(dbus_data->ch_r_x) > 0.01) ?
                                      dbus_data->wheel * gyro_rotate_reduction_ :
                                      dbus_data->wheel);
  vel_cmd_sender_->setLinearXVel(is_gyro_ ? dbus_data->ch_r_y * gyro_move_reduction_ : dbus_data->ch_r_y);
  vel_cmd_sender_->setLinearYVel(is_gyro_ ? -dbus_data->ch_r_x * gyro_move_reduction_ : -dbus_data->ch_r_x);

  if (shooter_cmd_sender_->getMsg()->mode != rm_msgs::ShootCmd::STOP)
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
}

void ChassisGimbalShooterManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updatePc(dbus_data);
  if (chassis_cmd_sender_->power_limit_->getState() != rm_common::PowerLimit::BURST && !is_gyro_ && !is_balance_)
  {  // Capacitor enter fast charge when chassis stop.
    if (!dbus_data->key_shift && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW &&
        std::sqrt(std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) >
            0.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    else if (chassis_power_ < 6.0 && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  }
}

void ChassisGimbalShooterManual::rightSwitchDownRise()
{
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::leftSwitchDownRise()
{
  ChassisGimbalManual::leftSwitchDownRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::leftSwitchMidRise()
{
  ChassisGimbalManual::leftSwitchMidRise();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterManual::leftSwitchMidOn(ros::Duration duration)
{
  if (track_data_.id == 0)
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  else
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
}

void ChassisGimbalShooterManual::leftSwitchUpRise()
{
  ChassisGimbalManual::leftSwitchUpRise();
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  last_shoot_freq_ = shooter_cmd_sender_->getShootFrequency();
}

void ChassisGimbalShooterManual::leftSwitchUpFall()
{
  shooter_cmd_sender_->setShootFrequency(last_shoot_freq_);
}

void ChassisGimbalShooterManual::leftSwitchUpOn(ros::Duration duration)
{
  if (track_data_.id == 0)
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  else
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  if (duration > ros::Duration(1.))
  {
    shooter_cmd_sender_->setShootFrequency(last_shoot_freq_);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(ros::Time::now());
  }
  else if (duration < ros::Duration(0.02))
  {
    shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::MINIMAL);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(ros::Time::now());
  }
  else
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterManual::mouseLeftPress()
{
  if (shooter_cmd_sender_->getMsg()->mode == rm_msgs::ShootCmd::STOP)
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    prepare_shoot_ = false;
  }
  if (prepare_shoot_)
  {
    if (!mouse_right_event_.getState() || (mouse_right_event_.getState() && track_data_.id != 0))
    {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_cmd_sender_->checkError(ros::Time::now());
    }
    else
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
}

void ChassisGimbalShooterManual::mouseRightPress()
{
  if (track_data_.id == 0)
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  else
  {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
  }
  if (switch_armor_target_srv_->getArmorTarget() == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
  {
    if (shooter_cmd_sender_->getMsg()->mode != rm_msgs::ShootCmd::STOP)
    {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_cmd_sender_->checkError(ros::Time::now());
    }
  }
}

void ChassisGimbalShooterManual::ePress()
{
  switch_armor_target_srv_->setArmorTargetType(rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE);
  switch_armor_target_srv_->callService();
  shooter_cmd_sender_->setArmorType(switch_armor_target_srv_->getArmorTarget());
}

void ChassisGimbalShooterManual::eRelease()
{
  switch_armor_target_srv_->setArmorTargetType(rm_msgs::StatusChangeRequest::ARMOR_ALL);
  switch_armor_target_srv_->callService();
  shooter_cmd_sender_->setArmorType(switch_armor_target_srv_->getArmorTarget());
}

void ChassisGimbalShooterManual::cPress()
{
  if (is_gyro_)
  {
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  else
  {
    setChassisMode(rm_msgs::ChassisCmd::RAW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::bPress()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void ChassisGimbalShooterManual::bRelease()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
}

void ChassisGimbalShooterManual::rPress()
{
  if (robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO)
  {
    if (camera_switch_cmd_sender_)
      camera_switch_cmd_sender_->switchCamera();
    if (scope_cmd_sender_)
    {
      use_scope_ = !scope_cmd_sender_->getState();
      if (use_scope_)
        gimbal_cmd_sender_->setEject(true);
      else
      {
        gimbal_cmd_sender_->setEject(false);
        adjust_image_transmission_ = false;
      }
    }
  }
}

void ChassisGimbalShooterManual::gPress()
{
  shooter_cmd_sender_->dropSpeed();
}

void ChassisGimbalShooterManual::wPress()
{
  ChassisGimbalManual::wPress();
  if ((robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO) &&
      (gimbal_cmd_sender_->getEject() && !use_scope_))
  {
    gimbal_cmd_sender_->setEject(false);
    manual_to_referee_pub_data_.hero_eject_flag = gimbal_cmd_sender_->getEject();
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::aPress()
{
  ChassisGimbalManual::aPress();
  if ((robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO) &&
      (gimbal_cmd_sender_->getEject() && !use_scope_))
  {
    gimbal_cmd_sender_->setEject(false);
    manual_to_referee_pub_data_.hero_eject_flag = gimbal_cmd_sender_->getEject();
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::sPress()
{
  ChassisGimbalManual::sPress();
  if ((robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO) &&
      (gimbal_cmd_sender_->getEject() && !use_scope_))
  {
    gimbal_cmd_sender_->setEject(false);
    manual_to_referee_pub_data_.hero_eject_flag = gimbal_cmd_sender_->getEject();
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::dPress()
{
  ChassisGimbalManual::dPress();
  if ((robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO) &&
      (gimbal_cmd_sender_->getEject() && !use_scope_))
  {
    gimbal_cmd_sender_->setEject(false);
    manual_to_referee_pub_data_.hero_eject_flag = gimbal_cmd_sender_->getEject();
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::wRelease()
{
  ChassisGimbalManual::wRelease();
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0);
}
void ChassisGimbalShooterManual::aRelease()
{
  ChassisGimbalManual::aRelease();
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0);
}
void ChassisGimbalShooterManual::sRelease()
{
  ChassisGimbalManual::sRelease();
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0);
}
void ChassisGimbalShooterManual::dRelease()
{
  ChassisGimbalManual::dRelease();
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0);
}
void ChassisGimbalShooterManual::wPressing()
{
  ChassisGimbalManual::wPressing();
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalShooterManual::aPressing()
{
  ChassisGimbalManual::aPressing();
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalShooterManual::sPressing()
{
  ChassisGimbalManual::sPressing();
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalShooterManual::dPressing()
{
  ChassisGimbalManual::dPressing();
  vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0);
}

void ChassisGimbalShooterManual::xPress()
{
  double roll{}, pitch{}, yaw{};
  try
  {
    quatToRPY(tf_buffer_.lookupTransform("odom", "yaw", ros::Time(0)).transform.rotation, roll, pitch, yaw);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
  gimbal_cmd_sender_->setGimbalTraj(yaw + M_PI, pitch);
}

void ChassisGimbalShooterManual::xRelease()
{
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalShooterManual::vPress()
{
  shooter_cmd_sender_->raiseSpeed();
}

void ChassisGimbalShooterManual::shiftPress()
{
  if (chassis_cmd_sender_->getMsg()->mode != rm_msgs::ChassisCmd::FOLLOW && is_gyro_)
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void ChassisGimbalShooterManual::shiftRelease()
{
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::RAW ||
      std::sqrt(std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) >
          0.0)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  else
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void ChassisGimbalShooterManual::ctrlVPress()
{
  if (shooter_cmd_sender_->getShootFrequency() != rm_common::HeatLimit::LOW)
    shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::LOW);
  else
    shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::HIGH);
}

void ChassisGimbalShooterManual::ctrlRPress()
{
  if (image_transmission_cmd_sender_)
    adjust_image_transmission_ = !image_transmission_cmd_sender_->getState();
  if (adjust_image_transmission_)
  {
    double roll{}, pitch{}, yaw{};
    try
    {
      quatToRPY(tf_buffer_.lookupTransform("odom", "yaw", ros::Time(0)).transform.rotation, roll, pitch, yaw);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
    gimbal_cmd_sender_->setGimbalTraj(yaw, pitch - 0.64);
  }
}

void ChassisGimbalShooterManual::ctrlRRelease()
{
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalShooterManual::ctrlBPress()
{
  switch_detection_srv_->switchEnemyColor();
  switch_detection_srv_->callService();
}

void ChassisGimbalShooterManual::ctrlQPress()
{
  shooter_calibration_->reset();
  gimbal_calibration_->reset();
}

void ChassisGimbalShooterManual::robotRevive()
{
  setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  ManualBase::robotRevive();
}

}  // namespace rm_manual
