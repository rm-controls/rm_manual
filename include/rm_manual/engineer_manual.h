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
  void leftSwitchUpFall() {
    runStepQueue("ARM_TEST");
    trigger_change_ui_->update("step", "ARM_TEST");
  }
  void leftSwitchDownFall();
  void runStepQueue(const std::string &step_queue_name);
  void actionActiveCallback() { operating_mode_ = MIDDLEWARE; }
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr &feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState &state,
                          const rm_msgs::EngineerResultConstPtr &result);
  void ctrlCPress() { action_client_.cancelAllGoals(); }
  void ctrlRPress() {
    runStepQueue("RECOVER");
    trigger_change_ui_->update("step", "RECOVER");
  }
  void ctrlZPress() {
    runStepQueue("ARM_FOLD_LOWER");
    trigger_change_ui_->update("step", "ARM_FOLD_LOWER");
  }
  void ctrlBPress() {
    target_ = "BIG_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }
  void ctrlFPress() {
    target_ = "FLOOR_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }
  void ctrlXPress() {
    target_ = "EXCHANGE_";
    prefix_ = "";
    trigger_change_ui_->update("step", prefix_ + target_);
  }
  void ctrlGPress() {
    prefix_ = "GRASP_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }
  void ctrlSPress() {
    prefix_ = "STORAGE_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }
  void ctrlDPress() {
    prefix_ = "DRAG_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }
  void ctrlQPress() {
    runStepQueue(prefix_ + target_ + "PRE");
    trigger_change_ui_->update("step", prefix_ + target_ + "PRE");
  }
  void ctrlWPress() {
    runStepQueue(prefix_ + target_ + "PROC");
    trigger_change_ui_->update("step", prefix_ + target_ + "PROC");
  }
  void ctrlEPress() {
    runStepQueue(prefix_ + target_ + "AFTER");
    trigger_change_ui_->update("step", prefix_ + target_ + "AFTER");
  }
  void ctrlVPress() {
    target_ = "LIGHT_BAR_";
    trigger_change_ui_->update("step", prefix_ + target_);
  }

  void shiftWPress() {
    runStepQueue("GIMBAL_FORWARD_UPPER");
    trigger_change_ui_->update("step", "GIMBAL_FORWARD_UPPER");
  }
  void shiftSPress() {
    runStepQueue("GIMBAL_FORWARD_LOWER");
    trigger_change_ui_->update("step", "GIMBAL_FORWARD_LOWER");
  }
  void shiftCPress() { power_on_calibration_->reset(); }
  void shiftXPress() {
    data_.referee_.sendInteractiveData(0x0200, data_.referee_.referee_data_.robot_color_ == "blue"
                                               ? rm_common::RobotId::BLUE_SENTRY : rm_common::RED_SENTRY,
                                       data_.referee_.referee_data_.interactive_data.data_ == 0 ? 1 : 0);
  }

  void cPress();

  enum { MANUAL, MIDDLEWARE };
  std::string target_, prefix_;
  int operating_mode_;
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
  rm_common::CalibrationQueue *power_on_calibration_{}, *mast_calibration_{}, *arm_calibration_{};
  rm_common::JointPositionBinaryCommandSender *mast_command_sender_, *card_command_sender_;
  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_c_event_, ctrl_r_event_, ctrl_f_event_,
      ctrl_z_event_, ctrl_b_event_, ctrl_x_event_, ctrl_v_event_, ctrl_g_event_, ctrl_s_event_, ctrl_d_event_,
      ctrl_q_event_, ctrl_w_event_, ctrl_e_event_, shift_w_event_, shift_s_event_, shift_c_event_,shift_x_event_, c_event_;
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
