//
// Created by luohx on 7/20/20.
//

#ifndef RM_MANUAL_COMMON_DATA_H_
#define RM_MANUAL_COMMON_DATA_H_

#include <ros/ros.h>
#include <rm_referee/referee/referee.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <rm_msgs/ActuatorState.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackData.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rm_msgs/CapacityData.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/ManualToReferee.h>

namespace rm_manual
{
class Data
{
public:
  explicit Data(ros::NodeHandle& nh) : tf_listener_(tf_buffer_)
  {
  }

  ros::Subscriber joint_state_sub_;
  ros::Subscriber actuator_state_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber game_robot_status_sub_;
  ros::Subscriber game_robot_hp_sub_;
  ros::Subscriber power_heat_data_sub_;
  ros::Subscriber game_status_sub_;
  ros::Subscriber capacity_sub_;
  ros::Subscriber referee_sub_;

  ros::Publisher manual_to_referee_pub_;

  sensor_msgs::JointState joint_state_;
  rm_msgs::ActuatorState actuator_state_;
  rm_msgs::DbusData dbus_data_;
  rm_msgs::TrackData track_data_;
  rm_msgs::GimbalDesError gimbal_des_error_;
  nav_msgs::Odometry odom_;
  rm_msgs::GameRobotStatus game_robot_status_data_;
  rm_msgs::GameRobotHp game_robot_hp_data_;
  rm_msgs::PowerHeatData power_heat_data_data_;
  rm_msgs::GameStatus game_status_data_;
  rm_msgs::CapacityData capacity_data_;
  rm_msgs::Referee referee_sub_data_;
  rm_msgs::ManualToReferee manual_to_referee_pub_data_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace rm_manual
#endif  // RM_MANUAL_COMMON_DATA_H_
