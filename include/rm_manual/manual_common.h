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
#include <rm_common/ros_utilities.h>
#include "rm_common/ori_tool.h"
#include "rm_manual/fsm_data.h"

/**
 * Control FSM handles the FSM states from a higher level
 */
template<typename T>
class Manual {
 public:
  explicit Manual(ros::NodeHandle &nh);
// Runs the FSM logic and handles the state transitions and normal runs
  void run();
  virtual void ctrlQCallback();
  virtual void ctrlWCallback();
  virtual void qCallback();
  virtual void wCallback();
  virtual void eCallback();
  virtual void rCallback();
  virtual void aCallback();
  virtual void sCallback();
  virtual void dCallback();
  virtual void fCallback();
  virtual void gCallback();
  virtual void zCallback();
  virtual void xCallback();
  virtual void cCallback();
  virtual void vCallback();
  virtual void shiftCallback();

  uint8_t getShootSpeedCmd(int shoot_speed);

  void setArm(double linear_x, double linear_y, double linear_z,
              double angular_x, double angular_y, double angular_z, ros::Time now);
  void setChassis(uint8_t chassis_mode, double linear_x, double linear_y, double angular_z);
  void setGimbal(uint8_t gimbal_mode, double rate_yaw, double rate_pitch, uint8_t target_id, double bullet_speed);
  void setShoot(uint8_t shoot_mode, int shoot_speed, double shoot_hz, ros::Time now);

  void loadParam();
  ros::NodeHandle nh_;
  // Send related data to FsmState
  FsmData<T> data_;

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;

  bool passive_flag_=true;
  bool raw_flag_=false;
  bool rc_flag_=true;

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

  double safety_power_ = 0;
  bool have_power_manager_ = false;
};

#endif //SRC_RM_SOFTWARE_RM_DECISION_SRC_FSM_FSM_STATE_H_
