//
// Created by peter on 2021/5/24.
//

#ifndef RM_MANUAL_REFEREE_REFEREE_UI_H_
#define RM_MANUAL_REFEREE_REFEREE_UI_H_

#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>
#include "rm_manual/referee/referee.h"

namespace rm_manual {
class RefereeUi {
 public:
  RefereeUi(Referee *referee) : referee_(referee) {};
  void displayArmorInfo(const ros::Time &now);
  void displayCapInfo(GraphicOperateType graph_operate_type);
  void displayChassisInfo(uint8_t chassis_mode, bool burst_flag, GraphicOperateType graph_operate_type);
  void displayGimbalInfo(uint8_t gimbal_mode, GraphicOperateType graph_operate_type);
  void displayShooterInfo(uint8_t shooter_mode, bool burst_flag, GraphicOperateType graph_operate_type);
  void displayAttackTargetInfo(bool attack_base_flag, GraphicOperateType graph_operate_type);

 private:
  double getArmorPosition();

  Referee *referee_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_{};
  ros::Time last_update_armor0_time_, last_update_armor1_time_, last_update_armor2_time_, last_update_armor3_time_,
      last_update_cap_time_;
};
} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_REFEREE_UI_H_
