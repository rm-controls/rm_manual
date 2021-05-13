//
// Created by kiana on 2021/3/22.
//

#include "rm_manual/target_cost_function.h"

TargetCostFunction::TargetCostFunction(ros::NodeHandle &nh) {
  ros::NodeHandle cost_nh = ros::NodeHandle(nh, "target_cost_function");
  cost_nh.param("k_f", k_f_, 0.0);
  cost_nh.param("k_hp", k_hp_, 0.01);
  cost_nh.param("track_msg_timeout", track_msg_timeout_, 1.0);
  cost_nh.param("enemy_group", enemy_color_, std::string("error"));
  id_ = 0;
  time_interval_ = 0.01;
}

void TargetCostFunction::input(rm_msgs::TrackDataArray track_data_array, GameRobotHp robot_hp, bool only_attack_base) {
  double timeout_judge = (ros::Time::now() - track_data_array.header.stamp).toSec();
  if (timeout_judge > track_msg_timeout_) id_ = 0;
  else decideId(track_data_array, robot_hp, only_attack_base);
}

void TargetCostFunction::decideId(rm_msgs::TrackDataArray track_data_array,
                                  GameRobotHp robot_hp,
                                  bool only_attack_base) {
  int target_numbers = track_data_array.tracks.size();
  int id_temp;
  double cost_temp;
  decide_old_target_time_ = ros::Time::now();

  if (target_numbers) {
    for (int i = 0; i < target_numbers; i++) {
      cost_temp = calculateCost(track_data_array.tracks[i], robot_hp);
      if (cost_temp <= calculate_cost_) {
        // detective a target near than last target,change target
        calculate_cost_ = cost_temp;
        id_temp = track_data_array.tracks[i].id;
      }
      if (only_attack_base && track_data_array.tracks[i].id == 8) {
        // enter only attack base mode,can not detective base,choose to attack sentry if we can detective
        id_ = 8;
      }
      if (only_attack_base && track_data_array.tracks[i].id == 9) {
        // enter only attack base mode, detective base
        id_ = 9;
        break;
      }
    }


    //maybe we can consider frequency at this part if did not enter attack base mode
    if (!only_attack_base && id_temp != id_) {
      decide_new_target_time_ = ros::Time::now();
      time_interval_ = time_interval_ + (decide_new_target_time_ - decide_old_target_time_).toSec();
      double judge = calculate_cost_ + k_f_ / time_interval_;
      if (judge <= choose_cost_) {
        id_ = id_temp;
        time_interval_ = 0.0;
      }
      if (id_ == 0) id_ = id_temp;
    }
    calculate_cost_ = 1000000.0;
    choose_cost_ = (!only_attack_base && id_ == id_temp) ? calculate_cost_ : choose_cost_;

  } else id_ = 0;

}

double TargetCostFunction::calculateCost(rm_msgs::TrackData track_data, GameRobotHp robot_hp) {
  /*
  double delta_x_2 = pow(track_data.pose.position.x + time_interval_ * track_data.speed.linear.x, 2);
  double delta_y_2 = pow(track_data.pose.position.y + time_interval_ * track_data.speed.linear.y, 2);
  double delta_z_2 = pow(track_data.pose.position.z + time_interval_ * track_data.speed.linear.z, 2);
   */

  //not speed
  double delta_x_2 = pow(track_data.map2detection.position.x, 2);
  double delta_y_2 = pow(track_data.map2detection.position.y, 2);
  double delta_z_2 = pow(track_data.map2detection.position.z, 2);
  double distance = sqrt(delta_x_2 + delta_y_2 + delta_z_2);

  //Hp
  double hp_cost;
  if (enemy_color_ == "red") {
    if (track_data.id == 1) hp_cost = robot_hp.red_1_robot_HP;
    else if (track_data.id == 2) hp_cost = robot_hp.red_2_robot_HP;
    else if (track_data.id == 3) hp_cost = robot_hp.red_3_robot_HP;
    else if (track_data.id == 4) hp_cost = robot_hp.red_4_robot_HP;
    else if (track_data.id == 5) hp_cost = robot_hp.red_5_robot_HP;
    else if (track_data.id == 7) hp_cost = robot_hp.red_7_robot_HP;
    else hp_cost = 0.0;
  } else if (enemy_color_ == "blue") {
    if (track_data.id == 1) hp_cost = robot_hp.blue_1_robot_HP;
    else if (track_data.id == 2) hp_cost = robot_hp.blue_2_robot_HP;
    else if (track_data.id == 3) hp_cost = robot_hp.blue_3_robot_HP;
    else if (track_data.id == 4) hp_cost = robot_hp.blue_4_robot_HP;
    else if (track_data.id == 5) hp_cost = robot_hp.blue_5_robot_HP;
    else if (track_data.id == 7) hp_cost = robot_hp.blue_7_robot_HP;
    else hp_cost = 0.0;
  } else hp_cost = 0.0;

  double cost = distance - k_hp_ * hp_cost;

  return cost;
}

int TargetCostFunction::output() const {
  return id_;
}
