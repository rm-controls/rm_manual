//
// Created by qiayuan on 5/23/21.
//

#ifndef RM_MANUAL_ENGINEER_MANUAL_H_
#define RM_MANUAL_ENGINEER_MANUAL_H_
#include "rm_manual/chassis_gimbal_manual.h"

#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>

#include <rm_msgs/EngineerAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <utility>

namespace rm_manual {
class EngineerManual : public ChassisGimbalManual {
 public:
  explicit EngineerManual(ros::NodeHandle &nh)
      : ChassisGimbalManual(nh), operating_mode_(MANUAL), action_client_("/engineer_middleware/move_arm", true),
        left_switch_up_fall_event_(boost::bind(&EngineerManual::leftSwitchUpFall, this, _1)),
        left_switch_down_fall_event_(boost::bind(&EngineerManual::leftSwitchDownFall, this, _1)),
        ctrl_c_press_event_(boost::bind(&EngineerManual::ctrlCPress, this, _1)),
        ctrl_r_press_event_(boost::bind(&EngineerManual::ctrlRPress, this, _1)) {
    ROS_INFO("Waiting for middleware to start.");
    action_client_.waitForServer();
    ROS_INFO("Middleware started.");
    // Command sender
    ros::NodeHandle nh_card(nh, "card");
    card_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_card);
    ros::NodeHandle nh_mast(nh, "mast");
    mast_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_mast);
    ros::NodeHandle nh_stone_platform(nh, "stone_platform");
    stone_platform_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_stone_platform);

    // Calibration
    XmlRpc::XmlRpcValue rpc_value;
    nh.getParam("power_on_calibration", rpc_value);
    power_on_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
    nh.getParam("arm_calibration", rpc_value);
    arm_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  }
  void run() override {
    ChassisGimbalManual::run();
    arm_calibration_->update(ros::Time::now());
    power_on_calibration_->update(ros::Time::now());
  }

 private:
  void checkKeyboard() override {
    ChassisGimbalManual::checkKeyboard();
    ctrl_c_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
    ctrl_r_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  }
  void updateRc() override {
    ChassisGimbalManual::updateRc();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    left_switch_up_fall_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
    left_switch_down_fall_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN);
  }
  void sendCommand(const ros::Time &time) override {
    mast_command_sender_->sendCommand(time);
    if (operating_mode_ == MANUAL) {
      ChassisGimbalManual::sendCommand(time);
      card_command_sender_->sendCommand(time);
    }
  }
  void remoteControlTurnOff() override {
    ManualBase::remoteControlTurnOff();
    action_client_.cancelAllGoals();
  }
  void chassisOutputOn(ros::Duration /*duration*/) override {
    power_on_calibration_->reset();
    if (MIDDLEWARE)
      action_client_.cancelAllGoals();
  }
  void rightSwitchDown(ros::Duration time) override {
    ChassisGimbalManual::rightSwitchDown(time);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    action_client_.cancelAllGoals();
  }

    void rightSwitchMid(ros::Duration time) override {
      ChassisGimbalManual::rightSwitchMid(time);
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    }

    void rightSwitchUp(ros::Duration time) override {
      ChassisGimbalManual::rightSwitchUp(time);
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    }

    void leftSwitchUpFall(ros::Duration time) { /*runStepQueue("ARM_TEST");*/
      stone_platform_command_sender_->open();
    }

    void leftSwitchDownFall(ros::Duration time) { /*arm_calibration_->reset();*/
      stone_platform_command_sender_->close();
    }

    void runStepQueue(std::string step_queue_name) {
      rm_msgs::EngineerGoal goal;
      goal.step_queue_name = step_queue_name;
      if (action_client_.isServerConnected()) {
        if (operating_mode_ == MANUAL)
          action_client_.sendGoal(goal,
                                  boost::bind(&EngineerManual::actionDoneCallback, this, _1, _2),
                                  boost::bind(&EngineerManual::actionActiveCallback, this),
                                  boost::bind(&EngineerManual::actionFeedbackCb, this, _1));
        operating_mode_ = MIDDLEWARE;
    } else
        ROS_ERROR("Can not connect to middleware");
    }

    void actionActiveCallback() { operating_mode_ = MIDDLEWARE; }

    void actionFeedbackCb(const rm_msgs::EngineerFeedbackConstPtr &feedback) {}

    void actionDoneCallback(const actionlib::SimpleClientGoalState &state,
                            const rm_msgs::EngineerResultConstPtr &result) {
      ROS_INFO("Finished in state [%s]", state.toString().c_str());
      ROS_INFO("Result: %i", result->finish);
      operating_mode_ = MANUAL;
    }

    void ctrlCPress(ros::Duration /*duration*/) { action_client_.cancelAllGoals(); }

    void ctrlRPress(ros::Duration /*duration*/) { runStepQueue("RECOVER"); }

    enum {
        MANUAL, MIDDLEWARE
    };
    int operating_mode_;
    actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
    rm_common::CalibrationQueue *power_on_calibration_{}, *arm_calibration_{};
    rm_common::JointPositionBinaryCommandSender *mast_command_sender_, *card_command_sender_
    , *stone_platform_command_sender_;
    FallingInputEvent left_switch_up_fall_event_, left_switch_down_fall_event_;
    RisingInputEvent ctrl_c_press_event_;
    ActiveHighInputEvent ctrl_r_press_event_;
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
