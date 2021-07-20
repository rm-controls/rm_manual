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
      : ChassisGimbalManual(nh), action_client_("/engineer_middleware/move_arm", true),
        left_switch_up_fall_event_(boost::bind(&EngineerManual::leftSwitchUpFall, this, _1)) {
    ROS_INFO("Waiting for middleware to start.");
    action_client_.waitForServer();
    ROS_INFO("Middleware started.");
    // Command sender
    ros::NodeHandle nh_card(nh, "card");
    card_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_card);
    ros::NodeHandle nh_mast(nh, "mast");
    mast_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_mast);
    // Calibration
    XmlRpc::XmlRpcValue rpc_value;
    nh.getParam("arm_calibration", rpc_value);
    arm_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
    nh.getParam("power_on_calibration", rpc_value);
    power_on_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  }
  void run() override {
    ChassisGimbalManual::run();
    arm_calibration_->update(ros::Time::now());
    power_on_calibration_->update(ros::Time::now());
  }
  void updateRc() override {
    ChassisGimbalManual::updateRc();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    left_switch_up_fall_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
  }
 private:
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    card_command_sender_->sendCommand(time);
    mast_command_sender_->sendCommand(time);
  }
  void remoteControlTurnOn() override {
    ManualBase::remoteControlTurnOn();
  }
  void chassisOutputOn(ros::Duration /*duration*/) override {
    power_on_calibration_->reset();
  }
  void rightSwitchDown(ros::Duration time) override {
    ChassisGimbalManual::rightSwitchDown(time);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    if (has_send_step_list_) {
      action_client_.cancelAllGoals();
    }
  }
  void rightSwitchMid(ros::Duration time) override {
    ChassisGimbalManual::rightSwitchMid(time);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  }
  void rightSwitchUp(ros::Duration time) override {
    ChassisGimbalManual::rightSwitchUp(time);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  }
  void leftSwitchUpFall(ros::Duration time) {
    arm_calibration_->reset();
  }
  void sendStepList(uint8_t step_queue_id) {
    rm_msgs::EngineerActionGoal goal;
    goal.goal.step_queue_id = step_queue_id;
    if (action_client_.isServerConnected()) {
      if (!has_send_step_list_) {
        action_client_.sendGoal(goal.goal);
        has_send_step_list_ = true;
      } else if (has_send_step_list_ && action_client_.getResult()->finish) {
        has_send_step_list_ = false;
      }
    } else
      ROS_WARN("Can not connected with move arm server");
  }
  bool has_send_step_list_{};
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
  rm_common::CalibrationQueue *power_on_calibration_{}, *arm_calibration_{};
  FallingInputEvent left_switch_up_fall_event_;
  rm_common::JointPositionBinaryCommandSender *mast_command_sender_, *card_command_sender_;
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
