//
// Created by qiayuan on 5/23/21.
//

#pragma once

#include "rm_manual/chassis_gimbal_manual.h"

#include <utility>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <rm_common/decision/calibration_queue.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_msgs/EngineerAction.h>
#include <rm_msgs/EngineerUi.h>
#include <rm_msgs/ExchangerMsg.h>
#include <rm_msgs/MultiDofCmd.h>
#include <rm_msgs/GpioData.h>
#include <angles/angles.h>

namespace rm_manual
{
class EngineerManual : public ChassisGimbalManual
{
public:
  enum ControlMode
  {
    MANUAL,
    MIDDLEWARE
  };

  enum JointMode
  {
    SERVO,
    JOINT
  };

  enum GimbalMode
  {
    RATE,
    DIRECT
  };

  enum AutoExchangeProcess
  {
    FIND,
    PRE_ADJUST,
    MOVE,
    POST_ADJUST,
    FINISH
  };

  EngineerManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

private:
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void sendCommand(const ros::Time& time) override;
  void updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data);
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState& state, const rm_msgs::EngineerResultConstPtr& result);
  void runStepQueue(const std::string& step_queue_name);
  void initMode();
  void actionActiveCallback()
  {
    operating_mode_ = MIDDLEWARE;
  }
  void remoteControlTurnOff() override;
  void chassisOutputOn() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchUpRise() override;
  void leftSwitchUpFall();
  void leftSwitchDownFall();
  void leftSwitchDownRise() override;
  void ctrlQPress();
  void ctrlWPress();
  void ctrlEPress();
  void ctrlAPress();
  void ctrlSPress();
  void ctrlZPress();
  void ctrlZPressing();
  void ctrlZRelease();
  void ctrlXPress();
  void ctrlCPress();
  void ctrlDPress();
  void ctrlVRelease();
  void ctrlVPress();
  void ctrlBPress();
  void ctrlFPress();
  void ctrlGPress();
  void ctrlRPress();
  void shiftPressing();
  void shiftRelease();
  void shiftZPress();
  void shiftXPress();
  void shiftCPress();
  void shiftBPress();
  void shiftBRelease();
  void shiftGPress();
  void shiftRPressing();
  void shiftRRelease();
  void shiftVPress();
  void shiftVRelease();
  void shiftFPress();
  void rPress();
  void qPressing();
  void qRelease();
  void ePressing();
  void eRelease();
  void zPressing();
  void zRelease();
  void xPress();
  void cPressing();
  void cRelease();
  void bPressing();
  void bRelease();
  void vPressing();
  void vRelease();
  void fPress();
  void fRelease();
  void gPress();
  void gRelease();

  void mouseLeftRelease();
  void mouseRightRelease();
  void actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data) override;
  void exchangeCallback(const rm_msgs::ExchangerMsg::ConstPtr& data);
  void gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data);
  void stoneNumCallback(const std_msgs::String ::ConstPtr& data);
  void sendUi();

  void autoExchange();
  void find();
  void preAdjust();
  void move();
  void postAdjust();
  void finish();
  void autoSearch(bool enable_chassis, bool enable_gimbal);
  void autoPreAdjust();
  void autoPostAdjust();
  void autoMove();
  void isArmFinish();
  void autoGimbalSearch();
  void autoPreAdjustChassis();
  void autoPostAdjustChassis();
  bool isChassisFinish();
  void moveChassis();
  void setChassisTarget(double goal_x, double goal_y, double goal_yaw);

  bool change_flag_{ true }, is_exchange_{}, target_shape_{}, reversal_motion_{}, is_joint7_up_{ true },
      is_enter_auto_{ false }, is_search_finish_{ false }, is_pre_adjust_finish_{ false },
      is_post_adjust_finish_{ false }, is_move_finish_{ false }, is_need_post_adjust_{ false }, is_recorded_{ false },
      move_finish_{ false }, set_once_flag_{ false }, is_move_start_{ false }, pre_arm_start_{ false };
  int operating_mode_{}, servo_mode_{}, gimbal_mode_{}, stone_num_{}, max_temperature_{}, auto_exchange_process_{ 0 };
  double angular_z_scale_{}, gyro_scale_{}, gyro_low_scale_{}, gyro_normal_scale_{}, exchange_speed_scale_{},
      gyro_exchange_scale_{}, pitch_min_{}, pitch_max_{}, yaw_min_{}, yaw_max_{}, yaw_direct_{ 1. },
      pitch_direct_{ 1. }, search_angle_{}, chassis_search_vel_{}, gimbal_search_vel_{}, store_chassis_search_vel_{},
      store_gimbal_search_vel_{}, move_times_{}, lock_time_{}, chassis_pos_tolerance_{}, chassis_angular_tolerance_,
      x_start_value_{}, chassis_error_pos_{}, chassis_error_yaw_{}, pre_yaw_scales_{};
  std::string prefix_{}, root_{}, drag_state_{ "on" }, max_temperature_joint_{}, joint_temperature_{},
      reversal_state_{}, gripper_state_{};
  geometry_msgs::TransformStamped base2yaw_{}, yaw2pitch_{}, exchange2base_{}, link22base_{}, link32base_{},
      exchange2stone_{};
  geometry_msgs::PoseStamped chassis_target_{}, chassis_original_target_{};

  ros::Time start_time_;
  ros::Publisher engineer_ui_pub_;
  ros::Subscriber reversal_vision_sub_, exchange_sub_, gripper_state_sub_, stone_num_sub_;
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;

  rm_msgs::EngineerUi engineer_ui_;
  rm_msgs::GpioData gpio_state_;
  rm_common::Vel3DCommandSender* servo_command_sender_;
  rm_common::MultiDofCommandSender* reversal_command_sender_;
  rm_common::ServiceCallerBase<std_srvs::Empty>* servo_reset_caller_;
  rm_common::JointPositionBinaryCommandSender *drag_command_sender_, *joint7_command_sender_;
  rm_common::CalibrationQueue *calibration_gather_{}, *joint5_calibration_{};
  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_q_event_, ctrl_a_event_, ctrl_z_event_, ctrl_w_event_,
      ctrl_s_event_, ctrl_x_event_, ctrl_e_event_, ctrl_d_event_, ctrl_c_event_, ctrl_b_event_, ctrl_v_event_, z_event_,
      q_event_, e_event_, x_event_, c_event_, v_event_, b_event_, f_event_, shift_z_event_, shift_x_event_,
      shift_c_event_, shift_v_event_, shift_b_event_, shift_g_event_, ctrl_r_event_, shift_q_event_, shift_f_event_,
      shift_e_event_, ctrl_g_event_, shift_r_event_, ctrl_f_event_, shift_event_, g_event_, r_event_, mouse_left_event_,
      mouse_right_event_;
};
}  // namespace rm_manual
