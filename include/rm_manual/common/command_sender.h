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
    if (!nh.getParam("topic", topic_))
      ROS_ERROR("Topic name no defined (namespace: %s)", nh.getNamespace().c_str());
    queue_size_ = getParam(nh, "queue_size", 1);
    ros::NodeHandle root_nh;
    pub_ = root_nh.advertise<MsgType>(topic_, queue_size_);
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
  explicit TimeStampCommandSenderBase(ros::NodeHandle &nh, const Referee &referee) :
      CommandSenderBase<MsgType>(nh), referee_(referee) {}
  void sendCommand(ros::Time time) override {
    CommandSenderBase<MsgType>::msg_.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
 protected:
  const Referee &referee_;
};

class VelCommandSender : public CommandSenderBase<geometry_msgs::Twist> {
 public:
  explicit VelCommandSender(ros::NodeHandle &nh) : CommandSenderBase<geometry_msgs::Twist>(nh) {
    if (!nh.getParam("max_vel_x", max_vel_x_))
      ROS_ERROR("Max X velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_vel_y", max_vel_y_))
      ROS_ERROR("Max Y velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_vel_w", max_vel_w_))
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
  explicit ChassisCommandSender(ros::NodeHandle &nh, const Referee &referee)
      : TimeStampCommandSenderBase<rm_msgs::ChassisCmd>(nh, referee) {
    double accel_x, accel_y, accel_w;
    if (!nh.getParam("accel_x", accel_x))
      ROS_ERROR("Accel X no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("accel_y", accel_y))
      ROS_ERROR("Accel Y no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("accel_w", accel_w))
      ROS_ERROR("Accel W no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("safety_power", safety_power_))
      ROS_ERROR("safety power no defined (namespace: %s)", nh.getNamespace().c_str());
    msg_.accel.linear.x = accel_x;
    msg_.accel.linear.y = accel_y;
    msg_.accel.angular.z = accel_w;
  }
  void sendCommand(ros::Time time) override {
    if (referee_.is_open_)
      msg_.power_limit = referee_.referee_data_.game_robot_status_.chassis_power_limit;
    else
      msg_.power_limit = safety_power_;
    TimeStampCommandSenderBase<rm_msgs::ChassisCmd>::sendCommand(time);
  }
 private:
  double safety_power_{};
};

class GimbalCommandSender : public TimeStampCommandSenderBase<rm_msgs::GimbalCmd> {
 public:
  explicit GimbalCommandSender(ros::NodeHandle &nh, const Referee &referee) :
      TimeStampCommandSenderBase<rm_msgs::GimbalCmd>(nh, referee) {
    if (!nh.getParam("max_yaw_vel", max_yaw_rate_))
      ROS_ERROR("Max yaw velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("max_pitch_vel", max_pitch_vel_))
      ROS_ERROR("Max pitch velocity no defined (namespace: %s)", nh.getNamespace().c_str());
    cost_function_ = new TargetCostFunction(nh, referee);
  }

  void setRate(double scale_yaw, double scale_pitch) {
    msg_.rate_yaw = scale_yaw * max_yaw_rate_;
    msg_.rate_pitch = scale_pitch * max_pitch_vel_;
  }
  void setId(int id) {
    msg_.target_id = id;
  }
  void setBulletSpeed(int bullet_speed) {
    msg_.bullet_speed = bullet_speed;
  }
  void updateCost(const rm_msgs::TrackDataArray &track_data_array, bool base_only = false) {
    msg_.target_id = cost_function_->costFunction(track_data_array, base_only);
    if (msg_.target_id == 0)
      setMode(rm_msgs::GimbalCmd::RATE);
  }
  TargetCostFunction *cost_function_;
 private:
  double max_yaw_rate_{}, max_pitch_vel_{};
};

class ShooterCommandSender : public TimeStampCommandSenderBase<rm_msgs::ShootCmd> {
 public:
  explicit ShooterCommandSender(ros::NodeHandle &nh, const Referee &referee)
      : TimeStampCommandSenderBase<rm_msgs::ShootCmd>(nh, referee) {
    ros::NodeHandle limit_nh(nh, "heat_limit");
    heat_limit_ = new HeatLimit(limit_nh, referee_);
  }
  void setHz(double hz) { expect_hz_ = hz; }
  void setMagazine(bool is_open) { msg_.magazine = is_open; }
  void sendCommand(ros::Time time) override {
    msg_.speed = heat_limit_->getSpeedLimit();
    msg_.hz = heat_limit_->getHz();
    TimeStampCommandSenderBase<rm_msgs::ShootCmd>::sendCommand(time);
  }

 private:
  double expect_hz_{};
  HeatLimit *heat_limit_{};
};

}

#endif // RM_MANUAL_COMMON_COMMAND_SENDER_H_
