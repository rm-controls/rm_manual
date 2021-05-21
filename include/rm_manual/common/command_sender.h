//
// Created by qiayuan on 5/18/21.
//

#ifndef RM_MANUAL_COMMON_COMMAND_SENDER_H_
#define RM_MANUAL_COMMON_COMMAND_SENDER_H_
#include <type_traits>

#include <ros/ros.h>
#include <rm_common/ros_utilities.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/ShootCmd.h>
#include <nav_msgs/Odometry.h>
#include "heat_limit.h"
#include "target_cost_function.h"

namespace rm_manual {

template<class MsgType>
class CommandSenderBase {
 public:
  explicit CommandSenderBase(ros::NodeHandle &nh) {
    if (nh.getParam("topic", topic_))
      ROS_ERROR("Topic name no defined (namespace: %s)", nh.getNamespace().c_str());
    queue_size_ = getParam(nh, "queue_size", 1);
    pub_ = nh.advertise<MsgType>(topic_, queue_size_);
  }

  void setMode(int mode) { if (!std::is_same<MsgType, geometry_msgs::Twist>::value) msg_.mode = mode; }

  virtual void sendCommand(ros::Time time) { pub_.publish(msg_); }

  MsgType *getMsg() { return &msg_; }
 protected:
  std::string topic_;
  uint32_t queue_size_;
  ros::Publisher pub_;
  MsgType msg_;
};

template<class MsgType>
class TimeStampCommandSenderBase : public CommandSenderBase<MsgType> {
 public:
  explicit TimeStampCommandSenderBase(ros::NodeHandle &nh) : CommandSenderBase<MsgType>(nh) {}
  void sendCommand(ros::Time time) override {
    CommandSenderBase<MsgType>::msg_.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
};

class VelCommandSender : public CommandSenderBase<geometry_msgs::Twist> {
 public:
  explicit VelCommandSender(ros::NodeHandle &nh) : CommandSenderBase<geometry_msgs::Twist>(nh) {
    if (nh.getParam("max_vel_x", max_vel_x_))
      ROS_ERROR("Max X velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (nh.getParam("max_vel_y", max_vel_y_))
      ROS_ERROR("Max Y velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (nh.getParam("max_vel_w", max_vel_w_))
      ROS_ERROR("Max W velocity no defined (namespace: %s)", nh.getNamespace().c_str());
  }

  void setXVel(double scale) { msg_.linear.x = scale * max_vel_x_; };
  void setYVel(double scale) { msg_.linear.y = scale * max_vel_y_; };
  void setWVel(double scale) { msg_.angular.z = scale * max_vel_w_; };

  void setVel(double scale_x, double scale_y, double scale_w) {
    setXVel(scale_x);
    setYVel(scale_y);
    setWVel(scale_w);
  }

 private:
  double max_vel_x_{}, max_vel_y_{}, max_vel_w_{};
};

class ChassisCommandSender : public TimeStampCommandSenderBase<rm_msgs::ChassisCmd> {
 public:
  explicit ChassisCommandSender(ros::NodeHandle &nh) : TimeStampCommandSenderBase<rm_msgs::ChassisCmd>(nh) {
    double accel_x, accel_y, accel_w;
    if (nh.getParam("accel_x", accel_x))
      ROS_ERROR("Accel X no defined (namespace: %s)", nh.getNamespace().c_str());
    if (nh.getParam("accel_y", accel_y))
      ROS_ERROR("Accel Y no defined (namespace: %s)", nh.getNamespace().c_str());
    if (nh.getParam("accel_w", accel_w))
      ROS_ERROR("Accel W no defined (namespace: %s)", nh.getNamespace().c_str());
    msg_.accel.linear.x = accel_x;
    msg_.accel.linear.x = accel_x;
    msg_.accel.angular.z = accel_w;
  }
  void setAccel(double x, double y, double angular) {
    msg_.accel.linear.x = x;
    msg_.accel.linear.x = y;
    msg_.accel.angular.z = angular;
  }

  void setPowerLimit(double power_limit) { msg_.power_limit = power_limit; }
};

class GimbalCommandSender : public TimeStampCommandSenderBase<rm_msgs::GimbalCmd> {
 public:
  explicit GimbalCommandSender(ros::NodeHandle &nh, const Referee &referee) :
      TimeStampCommandSenderBase<rm_msgs::GimbalCmd>(nh) {
    ros::NodeHandle cost_nh(nh, "cost_function");
    if (nh.getParam("max_yaw_vel", max_yaw_rate_))
      ROS_ERROR("Max yaw velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (nh.getParam("max_pitch_vel", max_pitch_vel_))
      ROS_ERROR("Max pitch velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    cost_function_ = new TargetCostFunction(cost_nh, referee);
  }

  void sendCommand(ros::Time time) override {
//    msg_.target_id = cost_function_->costFunction();
    TimeStampCommandSenderBase<rm_msgs::GimbalCmd>::sendCommand(time);
  }
 private:
  double max_yaw_rate_{}, max_pitch_vel_{};
  TargetCostFunction *cost_function_;
};

class ShooterCommandSender : public TimeStampCommandSenderBase<rm_msgs::ShootCmd> {
 public:
  explicit ShooterCommandSender(ros::NodeHandle &nh, HeatLimit::Type type, const Referee &referee)
      : TimeStampCommandSenderBase<rm_msgs::ShootCmd>(nh), heat_limit_(nh, type, referee) {
  }
  void setSpeed(int speed) { msg_.speed = speed; }
  void setHz(double hz) { expect_hz_ = hz; }

  void sendCommand(ros::Time time) override {
    msg_.hz = heat_limit_.getHz(expect_hz_);
    TimeStampCommandSenderBase<rm_msgs::ShootCmd>::sendCommand(time);
  }

 private:
  double bullet_heat_{}, safe_shoot_frequency_{}, expect_hz_{};
  HeatLimit heat_limit_;
};

}

#endif // RM_MANUAL_COMMON_COMMAND_SENDER_H_
