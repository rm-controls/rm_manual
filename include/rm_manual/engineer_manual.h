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

namespace rm_manual {
class EngineerManual : public ChassisGimbalManual {
 public:
  explicit EngineerManual(ros::NodeHandle &nh);
  void run() override;
 private:
  void checkKeyboard() override;
  void updateRc() override;
  void updatePc() override;
  void sendCommand(const ros::Time &time) override;
  void drawUi(const ros::Time &time) override;
  void remoteControlTurnOff() override;
  void chassisOutputOn() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchUpFall() { runStepQueue("ARM_TEST"); }
  void leftSwitchDownFall();
  void runStepQueue(const std::string &step_queue_name);
  void actionActiveCallback() { operating_mode_ = MIDDLEWARE; }
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr &feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState &state,
                          const rm_msgs::EngineerResultConstPtr &result);
  void ctrlCPress() { action_client_.cancelAllGoals(); }
  void ctrlRPress() { runStepQueue("RECOVER"); }
  void ctrlFPress() { runStepQueue("ARM_FOLD_UPPER"); }
  void ctrlWPress() { runStepQueue("GRASP_BIG"); }
  void ctrlSPress() { runStepQueue("STORAGE"); }
  void ctrlQPress() { runStepQueue("PLACE"); }
  void cPress(ros::Duration duration);

  enum { MANUAL, MIDDLEWARE };
  int operating_mode_;
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
  rm_common::CalibrationQueue *power_on_calibration_{}, *mast_calibration_{}, *arm_calibration_{};
  rm_common::JointPositionBinaryCommandSender *mast_command_sender_, *card_command_sender_;
  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_c_event_, ctrl_f_event_, ctrl_r_event_, ctrl_w_event_,
      ctrl_s_event_, ctrl_q_event_, c_event_;
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
