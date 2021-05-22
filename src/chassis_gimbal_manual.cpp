//
// Created by qiayuan on 5/22/21.
//
#include "rm_manual/chassis_gimbal_manual.h"
void rm_manual::ChassisGimbalManual::sendCommand(const ros::Time &time) {
  if (have_power_manager_)
    chassis_cmd_sender_->setPowerLimit(data_.referee_->power_manager_data_.parameters[1]);
  else if (!(have_power_manager_)
      && data_.referee_->referee_data_.game_robot_status_.max_HP != 0)
    chassis_cmd_sender_->setPowerLimit(data_.referee_->referee_data_.game_robot_status_.chassis_power_limit);
  else
    chassis_cmd_sender_->setPowerLimit(safety_power_);
  chassis_cmd_sender_->sendCommand(time);
  vel_cmd_sender_->sendCommand(time);
  gimbal_cmd_sender_->sendCommand(time);
}
