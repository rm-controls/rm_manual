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
#include <rm_msgs/MultiDofCmd.h>
#include <rm_msgs/GpioData.h>
#include <rm_msgs/EngineerUi.h>

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

  enum SpeedMode
  {
    LOW,
    NORMAL,
    FAST,
    EXCHANGE
  };

  EngineerManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

private:
  void changeSpeedMode(SpeedMode speed_mode);
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data);
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data);
  void stoneNumCallback(const std_msgs::String ::ConstPtr& data);
  void sendCommand(const ros::Time& time) override;
  void runStepQueue(const std::string& step_queue_name);
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState& state, const rm_msgs::EngineerResultConstPtr& result);

  void initMode();
  void enterServo();
  void actionActiveCallback()
  {
    operating_mode_ = MIDDLEWARE;
  }
  void remoteControlTurnOff() override;
  void chassisOutputOn() override;
  void gimbalOutputOn() override;

  void rightSwitchUpRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchDownRise() override;
  void leftSwitchUpRise() override;
  void leftSwitchUpFall();
  void leftSwitchDownRise() override;
  void leftSwitchDownFall();
  void ctrlAPress();
  void ctrlBPress();
  void ctrlCPress();
  void ctrlDPress();
  void ctrlEPress();
  void ctrlFPress();
  void ctrlGPress();
  void ctrlQPress();
  void ctrlRPress();
  void ctrlSPress();
  void ctrlVPress();
  void ctrlVRelease();
  void ctrlWPress();
  void ctrlXPress();
  void ctrlZPress();

  void bPressing();
  void bRelease();
  void cPressing();
  void cRelease();
  void ePressing();
  void eRelease();
  void fPress();
  void fRelease();
  void gPress();
  void gRelease();
  void qPressing();
  void qRelease();
  void rPress();
  void vPressing();
  void vRelease();
  void xPress();
  void zPressing();
  void zRelease();

  void shiftPressing();
  void shiftRelease();
  void shiftBPress();
  void shiftBRelease();
  void shiftCPress();
  void shiftFPress();
  void shiftGPress();
  void shiftRPress();
  void shiftRRelease();
  void shiftVPress();
  void shiftVRelease();
  void shiftXPress();
  void shiftZPress();

  void mouseLeftRelease();
  void mouseRightRelease();

  // Servo

  bool change_flag_{};
  double angular_z_scale_{}, gyro_scale_{}, fast_gyro_scale_{}, low_gyro_scale_{}, normal_gyro_scale_{},
      exchange_gyro_scale_{}, fast_speed_scale_{}, low_speed_scale_{}, normal_speed_scale_{}, exchange_speed_scale_{};

  std::string prefix_{}, root_{}, reversal_state_{}, drag_state_{ "off" }, gripper_state_{ "off" };
  int operating_mode_{}, servo_mode_{}, gimbal_mode_{}, stone_num_{ 0 }, gimbal_height_{ 0 };

  ros::Time last_time_;
  ros::Subscriber stone_num_sub_, gripper_state_sub_;
  ros::Publisher engineer_ui_pub_;

  rm_msgs::GpioData gpio_state_;
  rm_msgs::EngineerUi engineer_ui_;

  rm_common::Vel3DCommandSender* servo_command_sender_;
  rm_common::ServiceCallerBase<std_srvs::Empty>* servo_reset_caller_;
  rm_common::JointPositionBinaryCommandSender *extend_arm_a_command_sender_, *extend_arm_b_command_sender_;
  rm_common::JointPointCommandSender *ore_bin_lifter_command_sender_, *ore_bin_rotate_command_sender_,
      *gimbal_lifter_command_sender_;
  rm_common::CalibrationQueue *calibration_gather_{}, *pitch_calibration_;

  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;

  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_a_event_, ctrl_b_event_, ctrl_c_event_, ctrl_d_event_,
      ctrl_e_event_, ctrl_f_event_, ctrl_g_event_, ctrl_q_event_, ctrl_r_event_, ctrl_s_event_, ctrl_v_event_,
      ctrl_w_event_, ctrl_x_event_, ctrl_z_event_, b_event_, c_event_, e_event_, f_event_, g_event_, q_event_, r_event_,
      v_event_, x_event_, z_event_, shift_event_, shift_b_event_, shift_c_event_, shift_e_event_, shift_f_event_,
      shift_g_event_, shift_v_event_, shift_q_event_, shift_r_event_, shift_x_event_, shift_z_event_, mouse_left_event_,
      mouse_right_event_;
};

}  // namespace rm_manual
