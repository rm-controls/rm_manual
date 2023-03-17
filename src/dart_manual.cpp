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
  qd_normal_[1] = getParam(nh_friction_left, "qd_1", 0.);
  qd_normal_[2] = getParam(nh_friction_left, "qd_2", 0.);
  qd_normal_[3] = getParam(nh_friction_left, "qd_3", 0.);
  qd_normal_[4] = getParam(nh_friction_left, "qd_4", 0.);
  qd_base_[1] = getParam(nh_friction_left, "qd_base_1", 0.);
  qd_base_[2] = getParam(nh_friction_left, "qd_base_2", 0.);
  qd_base_[3] = getParam(nh_friction_left, "qd_base_3", 0.);
  qd_base_[4] = getParam(nh_friction_left, "qd_base_4", 0.);
  qd_ = qd_normal_[1];
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
  right_switch_down_event_.setActiveHigh(boost::bind(&DartManual::rightSwitchDownOn, this));
  chassis_power_on_event_.setRising(boost::bind(&DartManual::chassisOutputOn, this));
  gimbal_power_on_event_.setRising(boost::bind(&DartManual::gimbalOutputOn, this));

  gpio_state_sub_ = nh.subscribe<rm_msgs::GpioData>("/controllers/gpio_controller/gpio_states", 10,
                                                    &DartManual::gpioStateCallback, this);
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &DartManual::dbusDataCallback, this);
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
  chassis_power_on_event_.update(data->mains_power_chassis_output);
  gimbal_power_on_event_.update(data->mains_power_gimbal_output);
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
  move(pitch_sender_, dbus_data->ch_r_y);
  move(yaw_sender_, dbus_data->ch_l_x);
}

void DartManual::checkReferee()
{
  ManualBase::checkReferee();
}

void DartManual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  gimbal_calibration_->reset();
}

void DartManual::leftSwitchDownRise()
{
  ManualBase::leftSwitchDownRise();
  trigger_calibration_->reset();
}

void DartManual::leftSwitchUpOn()
{
  ManualBase::leftSwitchUpOn();
  if (flag_ == 0)
  {
    start_ = ros::Time::now();
    flag_ = 1;
  }
  duration_ = ros::Time::now() - start_;
  if (duration_.toSec() > upward_time_.toSec())
    trigger_sender_->setPoint(0.);
  else
    trigger_sender_->setPoint(upward_vel_);
}

void DartManual::leftSwitchMidRise()
{
  ManualBase::leftSwitchMidRise();
  flag_ = 0;
  trigger_sender_->setPoint(0.);
}

void DartManual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  friction_right_sender_->setPoint(0.);
  friction_left_sender_->setPoint(0.);
}

void DartManual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  ROS_INFO("Ready to shooter");
  friction_right_sender_->setPoint(qd_);
  friction_left_sender_->setPoint(qd_);
}

void DartManual::rightSwitchDownOn()
{
  ManualBase::rightSwitchDownOn();
  recordPosition(data_);
  if (data_->ch_l_y == 1.)
  {
    pitch_sender_->setPoint(pitch_outpost_);
    yaw_sender_->setPoint(yaw_outpost_);
  }
  if (data_->ch_l_y == -1.)
  {
    pitch_sender_->setPoint(pitch_base_);
    yaw_sender_->setPoint(yaw_base_);
  }
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

void DartManual::recordPosition(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  if (dbus_data->ch_r_y == 1.)
  {
    pitch_outpost_ = joint_state_.position[pitch_sender_->getIndex()];
    yaw_outpost_ = joint_state_.position[yaw_sender_->getIndex()];
    ROS_INFO("recorded outpost position");
  }
  if (dbus_data->ch_r_y == -1.)
  {
    pitch_base_ = joint_state_.position[pitch_sender_->getIndex()];
    yaw_base_ = joint_state_.position[yaw_sender_->getIndex()];
    ROS_INFO("recorded base position");
  }
}
void DartManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  data_ = data;
  wheel_state_ = data->wheel;
  if (wheel_state_ == 1.0 || wheel_state_ == -1.0)
  {
    wheel_analog_level_ = 1;
    if ((wheel_analog_level_ - wheel_last_level_) > 0)
    {
      if(wheel_state_ == 1.0)
      {
        if (!wheel_flag_)
        {
          scale_ = scale_micro_;
          wheel_flag_ = !wheel_flag_;
        }
        else
        {
          scale_ = 0.04;
          wheel_flag_ = !wheel_flag_;
        }
      }
      if(wheel_state_ == -1.0)
      {
        if(!base_flag_)
        {
          speed_base_flag_ = 1;
          qd_ = qd_base_[state_];
          friction_right_sender_->setPoint(qd_);
          friction_left_sender_->setPoint(qd_);
          ROS_INFO("friction wheels : BASE_MODE");
          base_flag_ = !base_flag_;
        }
        else
        {
          qd_ = qd_normal_[state_];
          friction_right_sender_->setPoint(qd_);
          friction_left_sender_->setPoint(qd_);
          ROS_INFO("friction wheels : NORMAL_MODE");
          speed_base_flag_ = 0;
          base_flag_ = !base_flag_;
        }
      }
    }
  }
  else
  {
    wheel_analog_level_ = 0;
  }
  wheel_last_level_ = wheel_analog_level_;

}
void DartManual::gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data)
{
  door_state_2_ = data->gpio_state[2];
  door_state_3_ = data->gpio_state[3];
  door_state_4_ = data->gpio_state[4];
  if (door_state_2_ || door_state_3_ || door_state_4_)
  {
    analog_level_ = 1;
    if((analog_level_ - last_level_) > 0)
    {
      if (door_state_2_)
        state_ = 2;
      if (door_state_3_)
        state_ = 3;
      if (door_state_4_)
        state_ = 4;
      switch (state_)
      {
        case 1:
          qd_ = qd_normal_[1];
          if(speed_base_flag_)
          {
            qd_ = qd_base_[1];
          }
          break;
        case 2:
          if (!return_state_1_)
          {
            qd_ = qd_normal_[2];
            if(speed_base_flag_)
            {
              qd_ = qd_base_[2];
            }
          }
          else
          {
            qd_ = qd_normal_[1];
            state_ = 1;
            if(speed_base_flag_)
            {
              qd_ = qd_base_[1];
            }
          }
          return_state_1_ = !return_state_1_;
          break;
        case 3:
          qd_ = qd_normal_[3];
          if(speed_base_flag_)
          {
            qd_ = qd_base_[3];
          }
          break;
        case 4:
          qd_ = qd_normal_[4];
          if(speed_base_flag_)
          {
            qd_ = qd_base_[4];
          }
          break;
      }
      friction_right_sender_->setPoint(qd_);
      friction_left_sender_->setPoint(qd_);
    }
  }
  else
  {
    analog_level_ = 0;
  }
  last_level_ = analog_level_;
}

}  // namespace rm_manual
