//
// Created by qiayuan on 5/18/21.
//

#ifndef RM_MANUAL_COMMON_COMMAND_SENDER_H_
#define RM_MANUAL_COMMON_COMMAND_SENDER_H_
#include <ros/ros.h>
#include <rm_common/ros_utilities.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/ShootCmd.h>
#include <nav_msgs/Odometry.h>

namespace rm_manual {

template<class MsgType>
class CommandSenderBase {
 public:
  explicit CommandSenderBase(ros::NodeHandle &nh) {
    if (nh.getParam("topic", topic_))
      ROS_ERROR("Topic name no defined");
    queue_size_ = getParam(nh, "queue_size", 1);
    pub_ = nh.advertise<MsgType>(topic_, queue_size_);
  }

  void sendCommand(ros::Time time) {
    if (!std::is_same<MsgType, geometry_msgs::Twist>::value)
      msg_.stamp = time;
    pub_.publish(msg_);
  }

 protected:
  std::string topic_;
  uint32_t queue_size_;
  ros::Publisher pub_;
  MsgType msg_;
};

class ChassisCommandSender : public CommandSenderBase<rm_msgs::ChassisCmd> {
 public:
  explicit ChassisCommandSender(ros::NodeHandle &nh) : CommandSenderBase<rm_msgs::ChassisCmd>(nh) {
    double accel_x, accel_y, accel_angular;
    if (nh.getParam("accel_x", accel_x))
      ROS_ERROR("Accel X no defined");
    if (nh.getParam("accel_y", accel_y))
      ROS_ERROR("Accel Y no defined");
    if (nh.getParam("accel_w", accel_angular))
      ROS_ERROR("Accel W no defined");
    msg_.accel.linear.x = accel_x;
    msg_.accel.linear.x = accel_x;
    msg_.accel.angular.z = accel_angular;
  }
  void setMode(int mode) { msg_.mode = mode; }
  void setAccel(double x, double y, double angular) {
    msg_.accel.linear.x = x;
    msg_.accel.linear.x = y;
    msg_.accel.angular.z = angular;
  }

  void setPowerLimit(double power_limit) { msg_.power_limit = power_limit; }
};

class VelCommandSender : public CommandSenderBase<geometry_msgs::Twist> {
 public:
  explicit VelCommandSender(ros::NodeHandle &nh) : CommandSenderBase<geometry_msgs::Twist>(nh) {
    if (nh.getParam("max_vel_x", max_vel_x_))
      ROS_ERROR("Coefficient of velocity X no defined");
    if (nh.getParam("max_vel_y", max_vel_y_))
      ROS_ERROR("Coefficient of velocity Y no defined");
    if (nh.getParam("max_vel_w", max_vel_w_))
      ROS_ERROR("Coefficient of velocity W no defined");
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

}

#endif // RM_MANUAL_COMMON_COMMAND_SENDER_H_
