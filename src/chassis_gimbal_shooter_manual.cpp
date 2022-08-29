//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_manual.h"

namespace rm_manual
{
ChassisGimbalShooterManual::ChassisGimbalShooterManual(ros::NodeHandle& nh) : ChassisGimbalManual(nh)
{
  ros::NodeHandle shooter_nh(nh, "shooter");
  shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, track_data_, game_robot_status_data_,
                                                            power_heat_data_data_, referee_sub_data_);

  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("shooter_calibration", rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
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
  ChassisGimbalManual::checkReferee();
  shooter_power_on_event_.update(game_robot_status_data_.mains_power_shooter_output);
  self_inspection_event_.update(game_status_data_.game_progress == 2);
  game_start_event_.update(game_status_data_.game_progress == 4);

  manual_to_referee_pub_data_.power_limit_state = chassis_cmd_sender_->power_limit_->getState();
  manual_to_referee_pub_data_.shoot_frequency = shooter_cmd_sender_->getShootFrequency();
  manual_to_referee_pub_data_.gimbal_eject = gimbal_cmd_sender_->getEject();
  manual_to_referee_pub_data_.det_armor_target = switch_detection_srv_->getArmorTarget();
  manual_to_referee_pub_data_.det_color = switch_detection_srv_->getColor();
  manual_to_referee_pub_data_.det_exposure = switch_detection_srv_->getExposureLevel();
  manual_to_referee_pub_data_.det_target = switch_detection_srv_->getTarget();
  manual_to_referee_pub_data_.stamp = ros::Time::now();
}

void ChassisGimbalShooterManual::checkKeyboard()
{
  ChassisGimbalManual::checkKeyboard();
  e_event_.update(dbus_data_.key_e);
  c_event_.update(dbus_data_.key_c);
  g_event_.update(dbus_data_.key_g);
  q_event_.update((!dbus_data_.key_ctrl) & dbus_data_.key_q);
  f_event_.update(dbus_data_.key_f);
  b_event_.update((!dbus_data_.key_ctrl) & dbus_data_.key_b);
  x_event_.update(dbus_data_.key_x);
  ctrl_c_event_.update(dbus_data_.key_ctrl & dbus_data_.key_c);
  ctrl_v_event_.update(dbus_data_.key_ctrl & dbus_data_.key_v);
  ctrl_r_event_.update(dbus_data_.key_ctrl & dbus_data_.key_r);
  ctrl_b_event_.update(dbus_data_.key_ctrl & dbus_data_.key_b & !dbus_data_.key_shift);
  shift_event_.update(dbus_data_.key_shift & !dbus_data_.key_ctrl);
  ctrl_shift_b_event_.update(dbus_data_.key_ctrl & dbus_data_.key_shift & dbus_data_.key_b);
  mouse_left_event_.update(dbus_data_.p_l);
  mouse_right_event_.update(dbus_data_.p_r);
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

void ChassisGimbalShooterManual::updateRc()
{
  ChassisGimbalManual::updateRc();
}

void ChassisGimbalShooterManual::updatePc()
{
  ChassisGimbalManual::updatePc();
  if (chassis_cmd_sender_->power_limit_->getState() != rm_common::PowerLimit::CHARGE &&
      chassis_cmd_sender_->getMsg()->mode != rm_msgs::ChassisCmd::GYRO)
  {
    if (!dbus_data_.key_shift && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW &&
        std::sqrt(std::pow(vel_cmd_sender_->getMsg()->linear.x, 2) + std::pow(vel_cmd_sender_->getMsg()->linear.y, 2)) >
            0.0)
      chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    else if (capacity_data_.chassis_power < 1.0 && chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
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
  chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
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
    shooter_cmd_sender_->checkError(gimbal_des_error_, ros::Time::now());
  }
  else if (duration < ros::Duration(0.02))
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(gimbal_des_error_, ros::Time::now());
  }
  else
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterManual::mouseLeftPress()
{
  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  if (dbus_data_.p_r)
    shooter_cmd_sender_->checkError(gimbal_des_error_, ros::Time::now());
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
  if (!dbus_data_.key_ctrl && !dbus_data_.key_shift)
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void ChassisGimbalShooterManual::wPress()
{
  ChassisGimbalManual::wPress();
  if ((game_robot_status_data_.robot_id == rm_referee::RobotId::BLUE_HERO ||
       game_robot_status_data_.robot_id == rm_referee::RobotId::RED_HERO) &&
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
  if ((game_robot_status_data_.robot_id == rm_referee::RobotId::BLUE_HERO ||
       game_robot_status_data_.robot_id == rm_referee::RobotId::RED_HERO) &&
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
  if ((game_robot_status_data_.robot_id == rm_referee::RobotId::BLUE_HERO ||
       game_robot_status_data_.robot_id == rm_referee::RobotId::RED_HERO) &&
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
  if ((game_robot_status_data_.robot_id == rm_referee::RobotId::BLUE_HERO ||
       game_robot_status_data_.robot_id == rm_referee::RobotId::RED_HERO) &&
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
  if (game_robot_status_data_.robot_id == rm_referee::RobotId::BLUE_HERO ||
      game_robot_status_data_.robot_id == rm_referee::RobotId::RED_HERO)
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
  switch_detection_srv_->switchExposureLevel();
  switch_detection_srv_->callService();
}

}  // namespace rm_manual
