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
  void leftSwitchUpRise() override;
  void leftSwitchUpFall();
  void leftSwitchDownFall();
  void runStepQueue(const std::string& step_queue_name);
  void actionActiveCallback()
  {
    operating_mode_ = MIDDLEWARE;
  }
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState& state, const rm_msgs::EngineerResultConstPtr& result);
  void ctrlCPress();
  void ctrlBPress();
  void ctrlNPress();
  void ctrlJPress();
  void ctrlKPress();
  void ctrlLPress();
  void ctrlUPress();
  void ctrlIPress();
  void ctrlOPress();
  void ctrlPPress();
  void ctrlMPress();
  void ctrlFPress();
  void ctrlGPress();
  void ctrlQPress();
  void ctrlWPress();
  void ctrlEPress();
  void ctrlRPress();
  void ctrlTPress();

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
