//
// Created by luotinkai on 2022/7/15.
//

#include "rm_manual/dart_manual.h"

namespace rm_manual
{
DartManual::DartManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  ros::NodeHandle nh_yaw = ros::NodeHandle(nh, "yaw");
  ros::NodeHandle nh_left_pitch = ros::NodeHandle(nh, "left_pitch");
  yaw_sender_ = new rm_common::JointPointCommandSender(nh_yaw, joint_state_);
  pitch_sender_ = new rm_common::JointPointCommandSender(nh_left_pitch, joint_state_);
  ros::NodeHandle nh_trigger = ros::NodeHandle(nh, "trigger");
  ros::NodeHandle nh_friction_left = ros::NodeHandle(nh, "friction_left");
  ros::NodeHandle nh_friction_right = ros::NodeHandle(nh, "friction_right");
  XmlRpc::XmlRpcValue qd_outpost, qd_base, yaw_offset, yaw_offset_base;
  nh_friction_left.getParam("qd_outpost", qd_outpost);
  ROS_ASSERT(qd_outpost.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(qd_outpost.size() == 4);
  for (int i = 0; i < qd_outpost.size(); ++i)
    ROS_ASSERT(qd_outpost[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  nh_friction_left.getParam("qd_base", qd_base);
  ROS_ASSERT(qd_base.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(qd_base.size() == 4);
  for (int i = 0; i < qd_outpost.size(); ++i)
    ROS_ASSERT(qd_base[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  nh_yaw.getParam("yaw_offset", yaw_offset);
  ROS_ASSERT(yaw_offset.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(yaw_offset.size() == 4);
  for (int i = 0; i < yaw_offset.size(); ++i)
    ROS_ASSERT(yaw_offset[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  nh_yaw.getParam("yaw_offset_base", yaw_offset_base);
  ROS_ASSERT(yaw_offset_base.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(yaw_offset_base.size() == 4);
  for (int i = 0; i < yaw_offset_base.size(); ++i)
    ROS_ASSERT(yaw_offset_base[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  for (int i = 0; i < 4; i++)
  {
    qd_outpost_.push_back(qd_outpost[i]);
    qd_base_.push_back(qd_base[i]);
    yaw_offset_.push_back(yaw_offset[i]);
    yaw_offset_base_.push_back(yaw_offset_base[i]);
  }
  qd_ = qd_outpost_[0];

  launch_position_1_ = getParam(nh_trigger, "launch_position_1", 0.003251);
  launch_position_2_ = getParam(nh_trigger, "launch_position_2", 0.008298);
  launch_position_3_ = getParam(nh_trigger, "launch_position_3", 0.014457);
  pitch_position_outpost_ = getParam(nh_left_pitch, "pitch_position_outpost", 0.);
  pitch_position_base_ = getParam(nh_left_pitch, "pitch_position_base", 0.);
  yaw_position_outpost_ = getParam(nh_yaw, "yaw_position_outpost", 0.);
  yaw_position_base_ = getParam(nh_yaw, "yaw_position_base", 0.);
  scale_ = getParam(nh_left_pitch, "scale", 0.);
  scale_micro_ = getParam(nh_left_pitch, "scale_micro", 0.);
  upward_vel_ = getParam(nh_trigger, "upward_vel", 0.);
  trigger_sender_ = new rm_common::JointPointCommandSender(nh_trigger, joint_state_);
  friction_left_sender_ = new rm_common::JointPointCommandSender(nh_friction_left, joint_state_);
  friction_right_sender_ = new rm_common::JointPointCommandSender(nh_friction_right, joint_state_);
  XmlRpc::XmlRpcValue trigger_rpc_value, gimbal_rpc_value;
  nh.getParam("trigger_calibration", trigger_rpc_value);
  trigger_calibration_ = new rm_common::CalibrationQueue(trigger_rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", gimbal_rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_rpc_value, nh, controller_manager_);
  left_switch_up_event_.setActiveHigh(boost::bind(&DartManual::leftSwitchUpOn, this));
  left_switch_mid_event_.setActiveHigh(boost::bind(&DartManual::leftSwitchMidOn, this));
  left_switch_down_event_.setActiveHigh(boost::bind(&DartManual::leftSwitchDownOn, this));
  right_switch_down_event_.setActiveHigh(boost::bind(&DartManual::rightSwitchDownOn, this));
  right_switch_mid_event_.setRising(boost::bind(&DartManual::rightSwitchMidRise, this));
  right_switch_up_event_.setRising(boost::bind(&DartManual::rightSwitchUpRise, this));
  wheel_clockwise_event_.setRising(boost::bind(&DartManual::wheelClockwise, this));
  wheel_anticlockwise_event_.setRising(boost::bind(&DartManual::wheelAntiClockwise, this));
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &DartManual::dbusDataCallback, this);
  dart_client_cmd_sub_ = nh_referee.subscribe<rm_msgs::DartClientCmd>("dart_client_cmd_data", 10,
                                                                      &DartManual::dartClientCmdCallback, this);
  game_robot_hp_sub_ =
      nh_referee.subscribe<rm_msgs::GameRobotHp>("game_robot_hp", 10, &DartManual::gameRobotHpCallback, this);
  game_status_sub_ =
      nh_referee.subscribe<rm_msgs::GameStatus>("game_status", 10, &DartManual::gameStatusCallback, this);
}

void DartManual::run()
{
  ManualBase::run();
  trigger_calibration_->update(ros::Time::now());
  gimbal_calibration_->update(ros::Time::now());
}

void DartManual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  game_robot_status_ = *data;
}

void DartManual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  game_status_ = *data;
}

void DartManual::sendCommand(const ros::Time& time)
{
  friction_left_sender_->sendCommand(time);
  friction_right_sender_->sendCommand(time);
  trigger_sender_->sendCommand(time);
  pitch_sender_->sendCommand(time);
  yaw_sender_->sendCommand(time);
}

void DartManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  move(pitch_sender_, dbus_data->ch_r_y);
  move(yaw_sender_, dbus_data->ch_l_x);
}

void DartManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  getDartFiredNum();
  if (game_status_.game_progress == rm_msgs::GameStatus::IN_BATTLE)
  {
    switch (auto_state_)
    {
      case OUTPOST:
        pitch_sender_->setPoint(pitch_outpost_);
        yaw_sender_->setPoint(yaw_outpost_ + yaw_offset_[dart_fired_num_]);
        qd_ = qd_outpost_[dart_fired_num_];
        break;
      case BASE:
        pitch_sender_->setPoint(pitch_base_);
        yaw_sender_->setPoint(yaw_base_ + yaw_offset_base_[dart_fired_num_]);
        qd_ = qd_base_[dart_fired_num_];
        break;
    }
    friction_right_sender_->setPoint(qd_);
    friction_left_sender_->setPoint(qd_);
    if (last_dart_door_status_ - dart_client_cmd_.dart_launch_opening_status ==
        rm_msgs::DartClientCmd::OPENING_OR_CLOSING - rm_msgs::DartClientCmd::OPENED)
      dart_door_open_times_++;
    if (dart_client_cmd_.dart_launch_opening_status == rm_msgs::DartClientCmd::OPENED)
    {
      if (auto_state_ == OUTPOST && dart_fired_num_ <= 2)
      {
        ROS_INFO_STREAM("Shoot outpost first time.");
        launch_rest_flag_ = 0;
        launchTwoDart();
      }
      if (auto_state_ == OUTPOST && dart_fired_num_ >= 2 && dart_door_open_times_ > 1)
      {
        ROS_INFO_STREAM("Shoot outpost second time");
        launch_rest_flag_ = 1;
        launchTwoDart();
      }
      if (auto_state_ == BASE)
      {
        ROS_INFO_STREAM("Shoot base.");
        trigger_sender_->setPoint(upward_vel_);
        if (trigger_position_ >= 0.0183)
          trigger_sender_->setPoint(0.);
      }
    }
    last_dart_door_status_ = dart_client_cmd_.dart_launch_opening_status;
  }
  else
  {
    friction_right_sender_->setPoint(0.);
    friction_left_sender_->setPoint(0.);
  }
}

void DartManual::checkReferee()
{
  ManualBase::checkReferee();
}

void DartManual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  gimbal_calibration_->reset();
  trigger_calibration_->reset();
}

void DartManual::leftSwitchDownOn()
{
  ManualBase::leftSwitchDownOn();
  friction_right_sender_->setPoint(0.);
  friction_left_sender_->setPoint(0.);
  trigger_sender_->setPoint(-upward_vel_);
  triggerComeBackProtect();
}

void DartManual::leftSwitchMidOn()
{
  ManualBase::leftSwitchMidOn();
  dart_fired_num_ = 0;
  friction_right_sender_->setPoint(qd_);
  friction_left_sender_->setPoint(qd_);
  launch_rest_flag_ = 1;
  trigger_sender_->setPoint(0.);
}

void DartManual::leftSwitchUpOn()
{
  ManualBase::leftSwitchUpOn();
  getDartFiredNum();
  launchTwoDart();
  switch (manual_state_)
  {
    case OUTPOST:
      qd_ = qd_outpost_[dart_fired_num_];
      yaw_sender_->setPoint(yaw_outpost_ + yaw_offset_[dart_fired_num_]);
      break;
    case BASE:
      qd_ = qd_base_[dart_fired_num_];
      yaw_sender_->setPoint(yaw_base_ + yaw_offset_base_[dart_fired_num_]);
      break;
  }
  friction_right_sender_->setPoint(qd_);
  friction_left_sender_->setPoint(qd_);
}

void DartManual::rightSwitchDownOn()
{
  recordPosition(dbus_data_);
  if (dbus_data_.ch_l_y == 1.)
  {
    pitch_sender_->setPoint(pitch_outpost_);
    yaw_sender_->setPoint(yaw_outpost_);
  }
  if (dbus_data_.ch_l_y == -1.)
  {
    pitch_sender_->setPoint(pitch_base_);
    yaw_sender_->setPoint(yaw_base_);
  }
  if (dbus_data_.ch_l_x == 1.)
  {
    pitch_sender_->setPoint(pitch_position_outpost_);
    yaw_sender_->setPoint(yaw_position_outpost_);
  }
  if (dbus_data_.ch_l_x == -1.)
  {
    pitch_sender_->setPoint(pitch_position_base_);
    yaw_sender_->setPoint(yaw_position_base_);
  }
}

void DartManual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  dart_door_open_times_ = 0;
}

void DartManual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  pitch_sender_->setPoint(pitch_outpost_);
  yaw_sender_->setPoint(yaw_outpost_);
}

void DartManual::move(rm_common::JointPointCommandSender* joint, double ch)
{
  if (!joint_state_.position.empty())
  {
    double position = joint_state_.position[joint->getIndex()];
    if (ch != 0.)
    {
      joint->setPoint(position + ch * scale_);
      if_stop_ = true;
    }
    if (ch == 0. && if_stop_)
    {
      joint->setPoint(joint_state_.position[joint->getIndex()]);
      if_stop_ = false;
    }
  }
}

void DartManual::triggerComeBackProtect()
{
  if (trigger_position_ < 0.0003)
    trigger_sender_->setPoint(0.);
}

void DartManual::launchTwoDart()
{
  if (dart_fired_num_ < 2)
  {
    launch_rest_flag_ = 0;
    trigger_sender_->setPoint(upward_vel_);
  }
  if (dart_fired_num_ == 2 && launch_rest_flag_ == 0)
    trigger_sender_->setPoint(0.);
  if (dart_fired_num_ >= 2 && launch_rest_flag_ == 1)
  {
    trigger_sender_->setPoint(upward_vel_);
    if (trigger_position_ >= 0.0183)
      trigger_sender_->setPoint(0.);
  }
}

void DartManual::getDartFiredNum()
{
  if (trigger_position_ < launch_position_1_)
    dart_fired_num_ = 0;
  if (trigger_position_ > launch_position_1_)
    dart_fired_num_ = 1;
  if (trigger_position_ > launch_position_2_)
    dart_fired_num_ = 2;
  if (trigger_position_ > launch_position_3_)
    dart_fired_num_ = 3;
}
void DartManual::recordPosition(const rm_msgs::DbusData dbus_data)
{
  if (dbus_data.ch_r_y == 1.)
  {
    pitch_outpost_ = joint_state_.position[pitch_sender_->getIndex()];
    yaw_outpost_ = joint_state_.position[yaw_sender_->getIndex()];
    ROS_INFO("Recorded outpost position.");
  }
  if (dbus_data.ch_r_y == -1.)
  {
    pitch_base_ = joint_state_.position[pitch_sender_->getIndex()];
    yaw_base_ = joint_state_.position[yaw_sender_->getIndex()];
    ROS_INFO("Recorded base position.");
  }
}
void DartManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  if (remote_is_open_)
    trigger_position_ = std::abs(joint_state_.position[trigger_sender_->getIndex()]);
  wheel_clockwise_event_.update(data->wheel == 1.0);
  wheel_anticlockwise_event_.update(data->wheel == -1.0);
  dbus_data_ = *data;
}

void DartManual::dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data)
{
  dart_client_cmd_ = *data;
}

void DartManual::gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
{
  switch (game_robot_status_.robot_id)
  {
    case rm_msgs::GameRobotStatus::RED_DART:
      outpost_hp_ = data->blue_outpost_hp;
      break;
    case rm_msgs::GameRobotStatus::BLUE_DART:
      outpost_hp_ = data->red_outpost_hp;
      break;
  }
  if (outpost_hp_ != 0)
    auto_state_ = OUTPOST;
  else
    auto_state_ = BASE;
}

void DartManual::wheelClockwise()
{
  switch (move_state_)
  {
    case NORMAL:
      scale_ = scale_micro_;
      move_state_ = MICRO;
      ROS_INFO("Pitch and yaw : MICRO_MOVE_MODE");
      break;
    case MICRO:
      scale_ = 0.04;
      move_state_ = NORMAL;
      ROS_INFO("Pitch and yaw : NORMAL_MOVE_MODE");
      break;
  }
}

void DartManual::wheelAntiClockwise()
{
  switch (manual_state_)
  {
    case OUTPOST:
      manual_state_ = BASE;
      qd_ = qd_base_[dart_fired_num_];
      ROS_INFO("Friction wheels : BASE_MODE");
      pitch_sender_->setPoint(pitch_base_);
      yaw_sender_->setPoint(yaw_base_);
      break;
    case BASE:
      manual_state_ = OUTPOST;
      qd_ = qd_outpost_[dart_fired_num_];
      ROS_INFO("Friction wheels : OUTPOST_MODE");
      pitch_sender_->setPoint(pitch_outpost_);
      yaw_sender_->setPoint(yaw_outpost_);
      break;
  }
}
}  // namespace rm_manual
