//
// Created by peter on 2020/12/3.
//

#ifndef SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
#define SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_

#include <iostream>
#include <queue>
#include <utility>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <controller_manager_msgs/SwitchController.h>

#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include "data.h"
#include "controller_manager.h"
#include "command_sender.h"

class Manual {
 public:
  explicit Manual(ros::NodeHandle &nh);
  void run();
  virtual void leftSwitchDown();
  virtual void leftSwitchMid();
  virtual void leftSwitchUp();
  virtual void rightSwitchDown();
  virtual void rightSwitchMid();
  virtual void rightSwitchUp();
  virtual void ctrlQPress();
  virtual void ctrlWPress();
  virtual void qPress(ros::Duration period);
  virtual void wPress(ros::Duration period);
  virtual void ePress(ros::Duration period);
  virtual void rPress(ros::Duration period);
  virtual void aPress(ros::Duration period);
  virtual void sPress(ros::Duration period);
  virtual void dPress(ros::Duration period);
  virtual void fPress(ros::Duration period);
  virtual void gPress(ros::Duration period);
  virtual void zPress(ros::Duration period);
  virtual void xPress(ros::Duration period);
  virtual void cPress(ros::Duration period);
  virtual void vPress(ros::Duration period);
  virtual void bPress(ros::Duration period);
  virtual void shiftPress(ros::Duration period);
  virtual void mouseLeftPress(ros::Duration period);
  virtual void mouseRightPress(ros::Duration period);
  virtual void mouseLeftRightPress(ros::Duration period);
  virtual void remoteControlTurnOff();
  virtual void remoteControlTurnOn();

  uint8_t getShootSpeedCmd(int shoot_speed);

  void setChassis(double linear_x, double linear_y, double angular_z);
  void setGimbal(uint8_t gimbal_mode, double rate_yaw, double rate_pitch, uint8_t target_id, double bullet_speed);
  void setShoot(uint8_t shoot_mode, int shoot_speed, double shoot_hz, ros::Time now);

  void loadParam();
  void powerLimit();

  ros::NodeHandle nh_;
  // Send related data to FsmState
  Data data_;

  ControllerManager *controller_manager_;

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;

  bool rc_flag_ = true;
  bool emergency_stop_ = true;
  bool remote_control_is_open_ = false;
  bool is_burst_ = false;
  bool only_attack_base_ = false;
  bool enter_pc_ = false;
  uint8_t current_chassis_mode_;
  uint8_t current_gimbal_mode_;
  uint8_t current_shooter_mode_;
  // chassis fsm control accelerate
  double accel_x_ = 0.0;
  double accel_y_ = 0.0;
  double accel_angular_ = 0.0;
  int brake_multiple_ = 1;

  // chassis fsm control coefficient
  double coefficient_x_ = 0.0;
  double coefficient_y_ = 0.0;
  double coefficient_angular_ = 0.0;

  // gimbal fsm control coefficient
  double coefficient_yaw_ = 0.0;
  double coefficient_pitch_ = 0.0;
  double gimbal_error_limit_ = 2.0;

  double expect_shoot_hz_ = 0.0;
  double safe_shoot_hz_ = 0.0;
  double safe_shoot_speed_ = 0;
  double actual_shoot_speed_ = 0;
  int ultimate_shoot_speed_ = 0;

  ros::Time last_press_q_ = ros::Time::now();
  ros::Time last_press_w_ = ros::Time::now();
  ros::Time last_press_e_ = ros::Time::now();
  ros::Time last_press_r_ = ros::Time::now();
  ros::Time last_press_t_ = ros::Time::now();
  ros::Time last_press_a_ = ros::Time::now();
  ros::Time last_press_s_ = ros::Time::now();
  ros::Time last_press_d_ = ros::Time::now();
  ros::Time last_press_f_ = ros::Time::now();
  ros::Time last_press_g_ = ros::Time::now();
  ros::Time last_press_z_ = ros::Time::now();
  ros::Time last_press_x_ = ros::Time::now();
  ros::Time last_press_c_ = ros::Time::now();
  ros::Time last_press_v_ = ros::Time::now();
  ros::Time last_press_b_ = ros::Time::now();
  ros::Time last_press_shift_ = ros::Time::now();
  ros::Time last_press_mouse_left_ = ros::Time::now();
  ros::Time last_press_mouse_right_ = ros::Time::now();
  ros::Time last_press_mouse_right_left_ = ros::Time::now();
  double safety_power_ = 0;
  bool have_power_manager_ = false;
  int last_target_id_ = 0;
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
