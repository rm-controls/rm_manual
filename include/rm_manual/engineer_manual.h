//
// Created by qiayuan on 5/23/21.
//

#pragma once

#include <rm_msgs/EngineerCmd.h>
#include <rm_msgs/EngineerAction.h>
#include "rm_manual/chassis_gimbal_manual.h"
#include <rm_common/decision/calibration_queue.h>

#include <utility>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>

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
  void remoteControlTurnOff() override;
  void chassisOutputOn() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchUpFall()
  {
    runStepQueue("ARM_TEST");
    engineer_cmd_data_.step_queue_name = "ARM_TEST";
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
  void ctrlRPress()
  {
    runStepQueue("RECOVER");
  }
  void ctrlZPress()
  {
    runStepQueue("ARM_FOLD_LOWER");
  }
  void ctrlBPress()
  {
    target_ = "BIG_";
  }
  void ctrlFPress()
  {
    target_ = "FLOOR_";
  }
  void ctrlXPress()
  {
    target_ = "EXCHANGE_";
    prefix_ = "";
  }
  void ctrlGPress()
  {
    prefix_ = "GRASP_";
  }
  void ctrlSPress()
  {
    prefix_ = "STORAGE_";
  }
  void ctrlDPress()
  {
    prefix_ = "DRAG_";
  }
  void ctrlQPress()
  {
    runStepQueue(prefix_ + target_ + "PRE");
  }
  void ctrlWPress()
  {
    runStepQueue(prefix_ + target_ + "PROC");
  }
  void ctrlEPress()
  {
    runStepQueue(prefix_ + target_ + "AFTER");
  }
  void ctrlVPress()
  {
    target_ = "LIGHT_BAR_";
  }

  void shiftWPress()
  {
    runStepQueue("GIMBAL_FORWARD_UPPER");
  }
  void shiftSPress()
  {
    runStepQueue("GIMBAL_FORWARD_LOWER");
    engineer_cmd_data_.step_queue_name = "RECOVER";
  }
  void shiftCPress()
  {
    power_on_calibration_->reset();
  }
  void shiftXPress()
  {
    sentry_mode_ = sentry_mode_ == 0 ? 1 : 0;
  }
  void cPress();

  ros::Publisher engineer_cmd_pub_;
  rm_msgs::EngineerCmd engineer_cmd_data_;
  enum
  {
    MANUAL,
    MIDDLEWARE
  };
  std::string target_, prefix_;
  int operating_mode_, sentry_mode_;
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
  rm_common::CalibrationQueue *power_on_calibration_{}, *mast_calibration_{}, *arm_calibration_{};
  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_c_event_, ctrl_r_event_, ctrl_f_event_, ctrl_z_event_,
      ctrl_b_event_, ctrl_x_event_, ctrl_v_event_, ctrl_g_event_, ctrl_s_event_, ctrl_d_event_, ctrl_q_event_,
      ctrl_w_event_, ctrl_e_event_, shift_w_event_, shift_s_event_, shift_c_event_, shift_x_event_, c_event_;
};

}  // namespace rm_manual
