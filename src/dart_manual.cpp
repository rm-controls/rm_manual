//
// Created by luotinkai on 2022/7/15.
//

#include "rm_manual/dart_manual.h"

namespace rm_manual
{
DartManual::DartManual(ros::NodeHandle& nh) : ManualBase(nh)
{
  ros::NodeHandle nh_yaw = ros::NodeHandle(nh, "yaw");
  ros::NodeHandle nh_left_pitch = ros::NodeHandle(nh, "left_pitch");
  yaw_sender_ = new rm_common::JointPointCommandSender(nh_yaw, data_.joint_state_);
  pitch_sender_ = new rm_common::JointPointCommandSender(nh_left_pitch, data_.joint_state_);

  ros::NodeHandle nh_trigger = ros::NodeHandle(nh, "trigger");
  ros::NodeHandle nh_friction_left = ros::NodeHandle(nh, "friction_left");
  ros::NodeHandle nh_friction_right = ros::NodeHandle(nh, "friction_right");
  qd_ = getParam(nh_friction_left, "qd_", 0.);
  forward_vel_ = getParam(nh_trigger, "forward_vel", 0.);
  trigger_sender_ = new rm_common::JointPointCommandSender(nh_trigger, data_.joint_state_);
  friction_left_sender_ = new rm_common::JointPointCommandSender(nh_friction_left, data_.joint_state_);
  friction_right_sender_ = new rm_common::JointPointCommandSender(nh_friction_right, data_.joint_state_);
  XmlRpc::XmlRpcValue trigger_rpc_value, gimbal_rpc_value;
  nh.getParam("trigger_calibration", trigger_rpc_value);
  trigger_calibration_ = new rm_common::CalibrationQueue(trigger_rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", gimbal_rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_rpc_value, nh, controller_manager_);

  left_switch_up_event_.setEdge(boost::bind(&DartManual::leftSwitchUpRise, this),
                                boost::bind(&DartManual::leftSwitchUpFall, this));
  chassis_power_on_event_.setRising(boost::bind(&DartManual::chassisOutputOn, this));
  gimbal_power_on_event_.setRising(boost::bind(&DartManual::gimbalOutputOn, this));
}

void DartManual::run()
{
  ManualBase::run();
  trigger_calibration_->update(ros::Time::now());
  gimbal_calibration_->update(ros::Time::now());
}

void DartManual::sendCommand(const ros::Time& time)
{
  friction_left_sender_->sendCommand(time);
  friction_right_sender_->sendCommand(time);
  trigger_sender_->sendCommand(time);
  pitch_sender_->sendCommand(time);
  yaw_sender_->sendCommand(time);
}

void DartManual::updateRc()
{
  ManualBase::updateRc();
  Move(pitch_sender_->getIndex(), data_.dbus_data_.ch_r_y, pitch_sender_);
  Move(yaw_sender_->getIndex(), data_.dbus_data_.ch_l_x, yaw_sender_);
}

void DartManual::updatePc()
{
  ManualBase::updatePc();
  recordPosition();
  if (data_.dbus_data_.ch_l_y == 1)
  {
    pitch_sender_->setPoint(pitch_post_);
    yaw_sender_->setPoint(yaw_post_);
  }
  if (data_.dbus_data_.ch_l_y == -1)
  {
    pitch_sender_->setPoint(pitch_base_);
    yaw_sender_->setPoint(yaw_base_);
  }
}

void DartManual::checkReferee()
{
  ManualBase::checkReferee();
  chassis_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_chassis_output_);
  gimbal_power_on_event_.update(data_.referee_.referee_data_.game_robot_status_.mains_power_gimbal_output_);
}

void DartManual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  gimbal_calibration_->reset();
}

void DartManual::leftSwitchDownRise()
{
  ManualBase::leftSwitchDownRise();
  friction_right_sender_->setPoint(0.);
  friction_left_sender_->setPoint(0.);
  trigger_calibration_->reset();
}

void DartManual::leftSwitchMidRise()
{
  ManualBase::leftSwitchMidRise();
  ROS_INFO("Ready to shooter");
  friction_right_sender_->setPoint(qd_);
  friction_left_sender_->setPoint(qd_);
}

void DartManual::leftSwitchUpRise()
{
  ManualBase::leftSwitchUpRise();
  trigger_sender_->setPoint(forward_vel_);
}

void DartManual::leftSwitchUpFall()
{
  trigger_sender_->setPoint(0.);
}

void DartManual::Move(int index, double ch_l, rm_common::JointPointCommandSender* joint)
{
  if (!data_.joint_state_.position.empty())
  {
    double position = data_.joint_state_.position[index];
    if (ch_l != 0)
    {
      joint->setPoint(position + ch_l * 0.04);
      set_position_ = true;
    }
    if (ch_l == 0 && set_position_)
    {
      joint->setPoint(data_.joint_state_.position[index]);
      set_position_ = false;
    }
  }
}

void DartManual::scaleAdjust()
{
  // Wait for test plot.
}

void DartManual::recordPosition()
{
  if (data_.dbus_data_.ch_r_y == 1)
  {
    pitch_post_ = data_.joint_state_.position[pitch_sender_->getIndex()];
    yaw_post_ = data_.joint_state_.position[yaw_sender_->getIndex()];
    ROS_INFO("recorded outpost position");
  }
  if (data_.dbus_data_.ch_r_y == -1)
  {
    pitch_base_ = data_.joint_state_.position[pitch_sender_->getIndex()];
    yaw_base_ = data_.joint_state_.position[yaw_sender_->getIndex()];
    ROS_INFO("recorded base position");
  }
}
}  // namespace rm_manual
