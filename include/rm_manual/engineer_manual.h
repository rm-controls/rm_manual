//
// Created by qiayuan on 5/23/21.
//

#pragma once

#include "rm_manual/chassis_gimbal_manual.h"

#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <rm_common/decision/calibration_queue.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/EngineerAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utility>
#include <rm_msgs/ReversalCmd.h>
#include <rm_msgs/EngineerUi.h>
namespace rm_manual
{
class EngineerManual : public ChassisGimbalManual
{
public:
  explicit EngineerManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

private:
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void sendCommand(const ros::Time& time) override;
  void updateServo();
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState& state, const rm_msgs::EngineerResultConstPtr& result);
  void runStepQueue(const std::string& step_queue_name);
  void judgePrefix();
  void judgeRoot();
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
  void ctrlQPress();     // LF small island
  void ctrlWPress();     // SKY island
  void ctrlEPress();     // RT small island
  void ctrlAPress();     // MID small island
  void ctrlSPress();     // MID big island
  void ctrlDPress();     // Ground stone
  void ctrlZPress();     // Drag back
  void ctrlXPress();     // Drag down
  void ctrlCPress();     // Cancel name
  void ctrlVPress();     // Gripper
  void ctrlBPress();     // HOME
  void ctrlFPress();     // Exchange
  void ctrlGPress();     // Store stone
  void ctrlRPress();     // Calibration
  void shiftPressing();  // low speed
  void shiftRelease();   // low speed
  void shiftQPress();    // Gimbal island
  void shiftQRelease();  //
  void shiftEPress();    //  Gimbal sky
  void shiftERelease();  //
  void shiftZPress();    //  Gimbal reversal
  void shiftXPress();    // Gimbal ground
  void shiftCPress();    //
  void shiftBPress();    // Gimbal back to home
  void shiftGPress();    // Take stone
  void shiftVPress();    // Gimbal Rate Mode
  void shiftVRelease();
  void rPress();    //
  void qPress();    // Turn left
  void ePress();    // Turn left
  void zPress();    // Servo z
  void zRelease();  //
  void xPress();    //
  void cPress();    // Servo z
  void cRelease();  //
  void bPress();    //
  void bRelease();  //
  void vPress();    // Reversal
  void vRelease();
  void fPress();             // Reversal
  void fRelease();           //
  void gPress();             // Reversal
  void gRelease();           //
  void mouseLeftRelease();   // execute next
  void mouseRightRelease();  // execute repeat
  void judgeReversal(double translate_err, int reversal_look, ros::Duration period);
  void visionCB(const rm_msgs::ReversalCmdConstPtr& msg);
  void updateUiDate(std::string step_name, std::string reversal_state, std::string drag_state, uint8_t stone_num);
  bool judgeUiChange(std::string step_name, std::string reversal_state, std::string drag_state, uint8_t stone_num);
  void sendUi(std::string step_name, std::string reversal_state, std::string drag_state, uint8_t stone_num);
  enum
  {
    MANUAL,
    MIDDLEWARE
  };
  enum
  {
    SERVO,
    JOINT
  };
  enum
  {
    RATE,
    DIRECT
  };

  rm_msgs::EngineerUi engineer_ui_;
  ros::Publisher ui_send_;
  realtime_tools::RealtimeBuffer<rm_msgs::ReversalCmd> reversal_rt_buffer_;
  rm_msgs::ReversalCmd data_reversal_;
  double translate_err_{};
  int reversal_look_{};
  double angular_z_scale_{};
  std::string prefix_, root_, reversal_state_, drag_state_, reversal_state_last_, drag_state_last_, step_name_last_;
  int operating_mode_{}, servo_mode_{}, gimbal_mode_{}, stone_num_{}, stone_num_last_{}, gripper_state_{};
  std::map<std::string, int> prefix_list_, root_list_;
  ros::Time last_time_;
  ros::Subscriber reversal_vision_sub_;
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
  rm_common::CalibrationQueue *power_on_calibration_{}, *arm_calibration_{};
  rm_common::ThreeSwitchCommandSender* drag_command_sender_;
  rm_common::ReversalCommandSender* reversal_command_sender_;
  rm_common::Vel3DCommandSender* servo_command_sender_;
  rm_common::ServiceCallerBase<std_srvs::Empty>* servo_reset_caller_;
  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_q_event_, ctrl_a_event_, ctrl_z_event_, ctrl_w_event_,
      ctrl_s_event_, ctrl_x_event_, ctrl_e_event_, ctrl_d_event_, ctrl_c_event_, ctrl_b_event_, ctrl_v_event_, z_event_,
      q_event_, e_event_, x_event_, c_event_, v_event_, b_event_, f_event_, shift_z_event_, shift_x_event_,
      shift_c_event_, shift_v_event_, shift_b_event_, shift_g_event_, ctrl_r_event_, shift_q_event_, shift_e_event_,
      ctrl_g_event_, shift_r_event_, ctrl_f_event_, shift_event_, g_event_, r_event_, mouse_left_event_,
      mouse_right_event_;
};

}  // namespace rm_manual
