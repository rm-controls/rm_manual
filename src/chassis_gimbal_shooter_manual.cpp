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

  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("shooter_calibration", rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);

  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &ChassisGimbalShooterManual::dbusDataCallback, this);
  game_robot_status_sub_ = nh_referee.subscribe<rm_msgs::GameRobotStatus>(
      "game_robot_status", 10, &ChassisGimbalShooterManual::gameRobotStatusCallback, this);
  game_status_sub_ = nh_referee.subscribe<rm_msgs::GameStatus>("game_status", 10,
                                                               &ChassisGimbalShooterManual::gameStatusCallback, this);
  power_heat_data_sub_ = nh_referee.subscribe<rm_msgs::PowerHeatData>(
      "power_heat_data", 10, &ChassisGimbalShooterManual::powerHeatDataCallback, this);
  gimbal_des_error_sub_ = nh.subscribe<rm_msgs::GimbalDesError>(
      "/controllers/gimbal_controller/error", 10, &ChassisGimbalShooterManual::gimbalDesErrorCallback, this);

  shooter_power_on_event_.setRising(boost::bind(&ChassisGimbalShooterManual::shooterOutputOn, this));
  self_inspection_event_.setRising(boost::bind(&ChassisGimbalShooterManual::selfInspectionStart, this));
  game_start_event_.setRising(boost::bind(&ChassisGimbalShooterManual::gameStart, this));
  left_switch_up_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::leftSwitchUpOn, this, _1));
  left_switch_mid_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::leftSwitchMidOn, this, _1));
  e_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ePress, this));
  c_event_.setRising(boost::bind(&ChassisGimbalShooterManual::cPress, this));
  q_event_.setRising(boost::bind(&ChassisGimbalShooterManual::qPress, this));
  f_event_.setRising(boost::bind(&ChassisGimbalShooterManual::fPress, this));
  b_event_.setRising(boost::bind(&ChassisGimbalShooterManual::bPress, this));
  ctrl_c_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlCPress, this));
  ctrl_v_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlVPress, this));
  ctrl_r_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlRPress, this));
  ctrl_b_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlBPress, this));
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
  shooter_cmd_sender_->computeTargetAcceleration();
  shooter_calibration_->update(ros::Time::now());
}

void ChassisGimbalShooterManual::checkReferee()
{
  manual_to_referee_pub_data_.power_limit_state = chassis_cmd_sender_->power_limit_->getState();
  manual_to_referee_pub_data_.shoot_frequency = shooter_cmd_sender_->getShootFrequency();
  manual_to_referee_pub_data_.gimbal_eject = gimbal_cmd_sender_->getEject();
  manual_to_referee_pub_data_.det_armor_target = switch_detection_srv_->getArmorTarget();
  manual_to_referee_pub_data_.det_color = switch_detection_srv_->getColor();
  manual_to_referee_pub_data_.det_exposure = switch_detection_srv_->getExposureLevel();
  manual_to_referee_pub_data_.det_target = switch_detection_srv_->getTarget();
  manual_to_referee_pub_data_.stamp = ros::Time::now();
  ChassisGimbalManual::checkReferee();
}

void ChassisGimbalShooterManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::checkKeyboard(dbus_data);
  e_event_.update(dbus_data->key_e);
  c_event_.update(dbus_data->key_c);
  g_event_.update(dbus_data->key_g);
  q_event_.update((!dbus_data->key_ctrl) & dbus_data->key_q);
  f_event_.update(dbus_data->key_f);
  b_event_.update((!dbus_data->key_ctrl && !dbus_data->key_shift) & dbus_data->key_b);
  x_event_.update(dbus_data->key_x);
  ctrl_c_event_.update(dbus_data->key_ctrl & dbus_data->key_c);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
  ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
  ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b & !dbus_data->key_shift);
  shift_event_.update(dbus_data->key_shift & !dbus_data->key_ctrl);
  ctrl_shift_b_event_.update(dbus_data->key_ctrl & dbus_data->key_shift & dbus_data->key_b);
  mouse_left_event_.update(dbus_data->p_l);
  mouse_right_event_.update(dbus_data->p_r);
}

void ChassisGimbalShooterManual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ChassisGimbalManual::gameRobotStatusCallback(data);
  shooter_cmd_sender_->updateGameRobotStatus(*data);
  shooter_power_on_event_.update(data->mains_power_shooter_output);
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

void ChassisGimbalShooterManual::trackCallback(const rm_msgs::TrackData::ConstPtr& data)
{
  ChassisGimbalManual::trackCallback(data);
  shooter_cmd_sender_->updateTrackData(*data);
}

void ChassisGimbalShooterManual::sendCommand(const ros::Time& time)
{
  ChassisGimbalManual::sendCommand(time);
  shooter_cmd_sender_->sendCommand(time);
}

void ChassisGimbalShooterManual::remoteControlTurnOff()
{
  ChassisGimbalManual::remoteControlTurnOff();
  shooter_cmd_sender_->setZero();
  shooter_calibration_->stop();
}

void ChassisGimbalShooterManual::remoteControlTurnOn()
{
  ChassisGimbalManual::remoteControlTurnOn();
  shooter_calibration_->stopController();
  std::string robot_color = robot_id_ >= 100 ? "blue" : "red";
  switch_detection_srv_->setEnemyColor(robot_id_, robot_color);
}

void ChassisGimbalShooterManual::robotDie()
{
  ManualBase::robotDie();
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
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

void ChassisGimbalShooterManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updateRc(dbus_data);
  if (shooter_cmd_sender_->getMsg()->mode != rm_msgs::ShootCmd::STOP)
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
}

void ChassisGimbalShooterManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updatePc(dbus_data);
  if (chassis_cmd_sender_->power_limit_->getState() != rm_common::PowerLimit::CHARGE &&
      chassis_cmd_sender_->getMsg()->mode != rm_msgs::ChassisCmd::GYRO)
  {
    if (!dbus_data->key_shift && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW &&
        std::sqrt(std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) >
            0.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    else if (chassis_power_ < 1.0 && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
}

void ChassisGimbalShooterManual::rightSwitchDownRise()
{
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
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
}

void ChassisGimbalShooterManual::leftSwitchUpOn(ros::Duration duration)
{
  if (track_data_.id == 0)
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  else
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  if (duration > ros::Duration(1.))
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(ros::Time::now());
  }
  else if (duration < ros::Duration(0.02))
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(ros::Time::now());
  }
  else
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterManual::mouseLeftPress()
{
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  shooter_cmd_sender_->checkError(ros::Time::now());
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
}

void ChassisGimbalShooterManual::ePress()
{
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::TWIST)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  else
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::TWIST);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::cPress()
{
  if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(0.0);
  }
  else
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(1.0);
  }
}

void ChassisGimbalShooterManual::bPress()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void ChassisGimbalShooterManual::wPress()
{
  ChassisGimbalManual::wPress();
  if ((robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO) &&
      gimbal_cmd_sender_->getEject())
  {
    gimbal_cmd_sender_->setEject(false);
    manual_to_referee_pub_data_.hero_eject_flag = gimbal_cmd_sender_->getEject();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::aPress()
{
  ChassisGimbalManual::aPress();
  if ((robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO) &&
      gimbal_cmd_sender_->getEject())
  {
    gimbal_cmd_sender_->setEject(false);
    manual_to_referee_pub_data_.hero_eject_flag = gimbal_cmd_sender_->getEject();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::sPress()
{
  ChassisGimbalManual::sPress();
  if ((robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO) &&
      gimbal_cmd_sender_->getEject())
  {
    gimbal_cmd_sender_->setEject(false);
    manual_to_referee_pub_data_.hero_eject_flag = gimbal_cmd_sender_->getEject();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::dPress()
{
  ChassisGimbalManual::dPress();
  if ((robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO) &&
      gimbal_cmd_sender_->getEject())
  {
    gimbal_cmd_sender_->setEject(false);
    manual_to_referee_pub_data_.hero_eject_flag = gimbal_cmd_sender_->getEject();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
}

void ChassisGimbalShooterManual::shiftPress()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void ChassisGimbalShooterManual::shiftRelease()
{
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
}

void ChassisGimbalShooterManual::ctrlCPress()
{
  switch_detection_srv_->switchArmorTargetType();
  switch_detection_srv_->callService();
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
  if (robot_id_ == rm_msgs::GameRobotStatus::BLUE_HERO || robot_id_ == rm_msgs::GameRobotStatus::RED_HERO)
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
    gimbal_cmd_sender_->setEject(true);
  }
  else
  {
    switch_detection_srv_->switchTargetType();
    switch_detection_srv_->callService();
    if (switch_detection_srv_->getTarget())
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  }
}

void ChassisGimbalShooterManual::ctrlBPress()
{
  switch_detection_srv_->switchEnemyColor();
  switch_detection_srv_->callService();
}

}  // namespace rm_manual
