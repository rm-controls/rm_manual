//
// Created by bruce on 2021/2/10.
//

#include "rm_manual/shooter_heat_limit.h"

void ShooterHeatLimit::input(Referee *referee, double expect_shoot_hz, double safe_shoot_hz) {
  uint16_t cooling_limit;
  uint16_t cooling_rate;
  uint16_t shooter_heat;
  double bullet_heat;

  if (referee->is_open_) { // using referee system
    if (referee->robot_id_ == kRedHero || referee->robot_id_ == kBlueHero) { // 42mm
      if (referee->referee_data_.game_robot_status_.shooter_id1_42mm_cooling_limit != 0) {
        cooling_limit = referee->referee_data_.game_robot_status_.shooter_id1_42mm_cooling_limit;
        shooter_heat = referee->referee_data_.power_heat_data_.shooter_id1_42mm_cooling_heat;
        bullet_heat = 100.0;

        if (shooter_heat < cooling_limit - bullet_heat) {
          hz = expect_shoot_hz;
        } else {
          hz = 0.0;
        }
      } else {
        hz = safe_shoot_hz;
      }
    } else { // 17mm
      if (referee->referee_data_.game_robot_status_.shooter_id1_17mm_cooling_limit != 0) {
        // get referee data
        cooling_limit = referee->referee_data_.game_robot_status_.shooter_id1_17mm_cooling_limit;
        cooling_rate = referee->referee_data_.game_robot_status_.shooter_id1_17mm_cooling_rate;
        shooter_heat = referee->referee_data_.power_heat_data_.shooter_id1_17mm_cooling_heat;
        bullet_heat = 10.0;

        // heat limit
        if (shooter_heat < cooling_limit - bullet_heat * 1.5) {
          hz = expect_shoot_hz;
        } else if (shooter_heat >= cooling_limit) {
          hz = 0.0;
        } else {
          hz = cooling_rate / bullet_heat;
        }
      } else {
        hz = safe_shoot_hz;
      }
    }
  } else {
    hz = safe_shoot_hz;
  }
}

double ShooterHeatLimit::output() const {
  return hz;
}
