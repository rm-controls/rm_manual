//
// Created by qiayuan on 5/23/21.
//

#ifndef RM_MANUAL_ENGINEER_MANUAL_H_
#define RM_MANUAL_ENGINEER_MANUAL_H_
#include "rm_manual/chassis_gimbal_manual.h"

#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <rm_common/decision/calibration_queue.h>
#include <std_msgs/Float64.h>
#include <rm_msgs/EngineerAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utility>

namespace rm_manual
{
class EngineerManual : public ChassisGimbalManual
{
public:
  explicit EngineerManual(ros::NodeHandle& nh);
  void run() override;

private:
  void checkKeyboard() override;
  void updateRc() override;
  void updatePc() override;
  void sendCommand(const ros::Time& time) override;
  void drawUi(const ros::Time& time) override;
  void remoteControlTurnOff() override;
  void chassisOutputOn() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchUpFall()
  {
    runStepQueue("BACK_HOME");
    trigger_change_ui_->update("step", "BACK_HOME");
  }
  void leftSwitchDownFall();
  void runStepQueue(const std::string& step_queue_name);
  void actionActiveCallback()
  {
    operating_mode_ = MIDDLEWARE;
  }
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState& state, const rm_msgs::EngineerResultConstPtr& result);
  void ctrlCPress()
  {
    action_client_.cancelAllGoals();
  }
  void ctrlBPress()
  {
    runStepQueue("BACK_HOME");
    trigger_change_ui_->update("step", "BACK_HOME");
  }
  void ctrlNPress()
  {
    runStepQueue("BACK_HOME2");
    trigger_change_ui_->update("step", "BACK_HOME2");
  }
  void ctrlJPress()
  {
    runStepQueue("EXCHANGE");
    trigger_change_ui_->update("step", "EXCHANGE");
  }
  void ctrlKPress()
  {
    runStepQueue("GROUND_STONE");
    trigger_change_ui_->update("step", "GROUND_STONE");
  }
  void ctrlLPress()
  {
    runStepQueue("SMALL_STONE");
    trigger_change_ui_->update("step", "SMALL_STONE");
  }
  void ctrlUPress()
  {
    runStepQueue("STORE");
    trigger_change_ui_->update("step", "STORE");
  }
  void ctrlIPress()
  {
    runStepQueue("GET_STONE");
    trigger_change_ui_->update("step", "GET_STONE");
  }
  void ctrlOPress()
  {
    runStepQueue("GET_STONE_SKY");
    trigger_change_ui_->update("step", "GET_STONE_SKY");
  }
  void ctrlPPress()
  {
    runStepQueue("GET_BARRIER");
    trigger_change_ui_->update("step", "GET_BARRIER");
  }
  void ctrlMPress()
  {
    runStepQueue("RELEASE_BARRIER");
    trigger_change_ui_->update("step", "RELEASE_BARRIER");
  }
  void ctrlFPress()
  {
    prefix_ = "GRASP_BIG";
    trigger_change_ui_->update("step", prefix_);
  }
  void ctrlGPress()
  {
    prefix_ = "SKY";
    trigger_change_ui_->update("step", prefix_);
  }
  void ctrlQPress()
  {
    toward_ = "_LF_";
    trigger_change_ui_->update("step", prefix_ + toward_);
  }
  void ctrlWPress()
  {
    toward_ = "_MID_";
    trigger_change_ui_->update("step", prefix_ + toward_);
  }
  void ctrlEPress()
  {
    toward_ = "_RT_";
    trigger_change_ui_->update("step", prefix_ + toward_);
  }
  void ctrlRPress()
  {
    runStepQueue(prefix_ + toward_ + "PRE");
    trigger_change_ui_->update("steps", prefix_ + toward_ + "PRE");
  }
  void ctrlTPress()
  {
    runStepQueue(prefix_ + toward_ + "PROC");
    trigger_change_ui_->update("steps", prefix_ + toward_ + "PROC");
  }

  void zPress();  // card long
  void xPress();  // card short
  void cPress();  // drag

  enum
  {
    MANUAL,
    MIDDLEWARE
  };
  std::string prefix_, toward_;
  int operating_mode_, sentry_mode_;
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
  rm_common::CalibrationQueue *power_on_calibration_{}, *arm_calibration_{};
  rm_common::JointPositionBinaryCommandSender* drag_command_sender_;
  rm_common::CardCommandSender* card_command_sender_;
  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_c_event_, ctrl_b_event_, ctrl_n_event_, ctrl_j_event_,
      ctrl_k_event_, ctrl_l_event_, ctrl_u_event_, ctrl_i_event_, ctrl_o_event_, ctrl_p_event_, ctrl_m_event_,
      ctrl_q_event_, ctrl_w_event_, ctrl_e_event_, ctrl_r_event_, ctrl_t_event_, ctrl_f_event_, ctrl_g_event_, z_event_,
      x_event_, c_event_;
};

}  // namespace rm_manual
#endif  // RM_MANUAL_ENGINEER_MANUAL_H_
