//
// Created by luohx on 7/20/20.
//

#ifndef RM_MANUAL_COMMON_DATA_H_
#define RM_MANUAL_COMMON_DATA_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <rm_common/decision/target_cost_function.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <rm_msgs/ActuatorState.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GimbalDesError.h>

#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/PowerHeatData.h>
#include <rm_msgs/CapacityData.h>

namespace rm_manual {

class Data {
public:
  explicit Data(ros::NodeHandle &nh) : tf_listener_(tf_buffer_) {
    // sub
    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>(
        "/joint_states", 10, &Data::jointStateCallback, this);
    actuator_state_sub_ = nh.subscribe<rm_msgs::ActuatorState>(
        "/actuator_states", 10, &Data::actuatorStateCallback, this);
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10,
                                                &Data::dbusDataCallback, this);
    track_sub_ = nh.subscribe<rm_msgs::TrackDataArray>(
        "/controllers/gimbal_controller/track", 10, &Data::trackCallback, this);
    gimbal_des_error_sub_ = nh.subscribe<rm_msgs::GimbalDesError>(
        "/controllers/gimbal_controller/error_des", 10,
        &Data::gimbalDesErrorCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 10,
                                                 &Data::odomCallback, this);
    game_robot_status_sub_ = nh.subscribe<rm_msgs::GameRobotStatus>(
        "/game_robot_status", 10, &Data::gameRobotStatusCallback, this);
    game_robot_hp_sub_ = nh.subscribe<rm_msgs::GameRobotHp>(
        "/game_robot_hp", 10, &Data::gameRobotHpCallback, this);
    game_status_sub_ = nh.subscribe<rm_msgs::GameStatus>(
        "/game_status", 10, &Data::gameStatusCallback, this);
    capacity_sub_ = nh.subscribe<rm_msgs::CapacityData>(
        "/capacity_data", 10, &Data::capacityDataCallback, this);
    power_heat_data_sub_ = nh.subscribe<rm_msgs::PowerHeatData>(
        "/power_heat_data", 10, &Data::powerHeatDataCallback, this);
    updateRefereeData();
  }

  void
  jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state) {
    joint_state_ = *joint_state;
  }
  void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr &data) {
    actuator_state_ = *data;
  }
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) {
    dbus_data_ = *data;
  }
  void trackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) {
    track_data_array_ = *data;
  }
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) {
    gimbal_des_error_ = *data;
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr &data) { odom_ = *data; }

  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr &data) {
    game_robot_status_data_ = *data;
  }
  void gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr &data) {
    game_robot_hp_data_ = *data;
  }
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr &data) {
    game_status_data_ = *data;
  }
  void capacityDataCallback(const rm_msgs::CapacityData ::ConstPtr &data) {
    capacity_data_ = *data;
  }
  void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr &data) {
    power_heat_data_data_ = *data;
  }

  void updateRefereeData() {
    referee_data_.game_robot_status_.mains_power_shooter_output_ =
        game_robot_status_data_.mains_power_shooter_output_;
    referee_data_.game_robot_status_.mains_power_gimbal_output_ =
        game_robot_status_data_.mains_power_gimbal_output_;
    referee_data_.game_robot_status_.mains_power_chassis_output_ =
        game_robot_status_data_.mains_power_chassis_output_;
    referee_data_.game_robot_status_.chassis_power_limit_ =
        game_robot_status_data_.chassis_power_limit_;
    referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_limit_ =
        game_robot_status_data_.shooter_id_1_17_mm_cooling_limit_;
    referee_data_.game_robot_status_.shooter_id_2_17_mm_cooling_limit_ =
        game_robot_status_data_.shooter_id_1_17_mm_cooling_limit_;
    referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_limit_ =
        game_robot_status_data_.shooter_id_1_17_mm_cooling_limit_;
    referee_data_.game_robot_status_.shooter_id_1_17_mm_cooling_rate_ =
        game_robot_status_data_.shooter_id_1_17_mm_cooling_rate_;
    referee_data_.game_robot_status_.shooter_id_2_17_mm_cooling_rate_ =
        game_robot_status_data_.shooter_id_2_17_mm_cooling_rate_;
    referee_data_.game_robot_status_.shooter_id_1_42_mm_cooling_rate_ =
        game_robot_status_data_.shooter_id_1_42_mm_cooling_rate_;
    referee_data_.game_robot_status_.shooter_id_1_17_mm_speed_limit_ =
        game_robot_status_data_.shooter_id_1_17_mm_speed_limit_;
    referee_data_.game_robot_status_.shooter_id_2_17_mm_speed_limit_ =
        game_robot_status_data_.shooter_id_2_17_mm_speed_limit_;
    referee_data_.game_robot_status_.shooter_id_1_42_mm_speed_limit_ =
        game_robot_status_data_.shooter_id_1_42_mm_speed_limit_;
    referee_data_.game_robot_status_.robot_level_ =
        game_robot_status_data_.robot_level_;
    referee_data_.game_robot_status_.robot_id_ =
        game_robot_status_data_.robot_id_;

    referee_data_.power_heat_data_.shooter_id_1_42_mm_cooling_heat_ =
        power_heat_data_data_.shooter_id_1_42_mm_cooling_heat_;
    referee_data_.power_heat_data_.shooter_id_1_17_mm_cooling_heat_ =
        power_heat_data_data_.shooter_id_1_42_mm_cooling_heat_;
    referee_data_.power_heat_data_.shooter_id_2_17_mm_cooling_heat_ =
        power_heat_data_data_.shooter_id_1_42_mm_cooling_heat_;

    referee_data_.capacity_data.chassis_power_ = capacity_data_.chassis_power_;
    referee_data_.capacity_data.is_online_ = capacity_data_.is_online_;
    referee_data_.capacity_data.limit_power_ = capacity_data_.limit_power_;
    referee_data_.capacity_data.cap_power_ = capacity_data_.cap_power_;

    referee_data_.game_robot_hp_.red_1_robot_hp_ =
        game_robot_hp_data_.red_1_robot_hp_;
    referee_data_.game_robot_hp_.red_2_robot_hp_ =
        game_robot_hp_data_.red_2_robot_hp_;
    referee_data_.game_robot_hp_.red_3_robot_hp_ =
        game_robot_hp_data_.red_3_robot_hp_;
    referee_data_.game_robot_hp_.red_4_robot_hp_ =
        game_robot_hp_data_.red_4_robot_hp_;
    referee_data_.game_robot_hp_.red_5_robot_hp_ =
        game_robot_hp_data_.red_5_robot_hp_;
    referee_data_.game_robot_hp_.red_7_robot_hp_ =
        game_robot_hp_data_.red_7_robot_hp_;

    referee_data_.game_robot_hp_.blue_1_robot_hp_ =
        game_robot_hp_data_.blue_1_robot_hp_;
    referee_data_.game_robot_hp_.blue_2_robot_hp_ =
        game_robot_hp_data_.blue_2_robot_hp_;
    referee_data_.game_robot_hp_.blue_3_robot_hp_ =
        game_robot_hp_data_.blue_3_robot_hp_;
    referee_data_.game_robot_hp_.blue_4_robot_hp_ =
        game_robot_hp_data_.blue_4_robot_hp_;
    referee_data_.game_robot_hp_.blue_5_robot_hp_ =
        game_robot_hp_data_.blue_5_robot_hp_;
    referee_data_.game_robot_hp_.blue_7_robot_hp_ =
        game_robot_hp_data_.blue_7_robot_hp_;
  };

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

  sensor_msgs::JointState joint_state_;
  rm_msgs::ActuatorState actuator_state_;
  rm_msgs::DbusData dbus_data_;
  rm_msgs::TrackDataArray track_data_array_;
  rm_msgs::GimbalDesError gimbal_des_error_;
  nav_msgs::Odometry odom_;
  rm_msgs::GameRobotStatus game_robot_status_data_;
  rm_msgs::GameRobotHp game_robot_hp_data_;
  rm_msgs::PowerHeatData power_heat_data_data_;
  rm_msgs::GameStatus game_status_data_;
  rm_msgs::CapacityData capacity_data_;

  rm_common::RefereeData referee_data_;
  serial::Serial serial_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}
#endif // RM_MANUAL_COMMON_DATA_H_
