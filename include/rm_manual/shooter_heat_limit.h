//
// Created by bruce on 2021/2/10.
//

#ifndef RM_MANUAL_INCLUDE_RM_MANUAL_SHOOTER_HEAT_LIMIT_H_
#define RM_MANUAL_INCLUDE_RM_MANUAL_SHOOTER_HEAT_LIMIT_H_

#include <ros/ros.h>
#include "rm_manual/referee.h"

class ShooterHeatLimit {
 public:
  ShooterHeatLimit() = default;
  void input(referee::Referee *referee, double expect_shoot_hz, double safe_shoot_hz);
  double output() const;

 private:
  double hz = 0.0;
};

#endif //RM_MANUAL_INCLUDE_RM_MANUAL_SHOOTER_HEAT_LIMIT_H_
