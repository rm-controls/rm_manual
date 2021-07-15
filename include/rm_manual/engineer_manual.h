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
      : ChassisGimbalManual(nh), action_client_("/engineer_middleware/move_arm", true) {
    ros::NodeHandle arm_servo(nh, "arm_servo");

    arm_servo_sender_ = new rm_common::Vel3DCommandSender(arm_servo);
    reset_servo_server_ = nh.serviceClient<std_srvs::Empty>("/servo_server/reset_servo_status");
    ROS_INFO("Waiting for move arm server to start.");
    action_client_.waitForServer();
    ROS_INFO("Move arm server started.");

    pub_ = nh.advertise<std_msgs::Float64>("/controllers/mast_controller/command", 1);
  }
 private:
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    arm_servo_sender_->sendCommand(time);
    pub_.publish(std_msgs::Float64());
  }
  void setZero() override {
    ChassisGimbalManual::setZero();
    arm_servo_sender_->setZero();
  }
  void rightSwitchMid() override {
    if (data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN)
      ChassisGimbalManual::rightSwitchMid();
  }
  void rightSwitchDown() override {
    ChassisGimbalManual::rightSwitchDown();
    if (has_send_step_list_) {
      action_client_.cancelAllGoals();
    }
  }
  void leftSwitchMid() override {
    rm_msgs::EngineerActionGoal g;
    sendStepList(g.goal.FOLD);
  }
  void leftSwitchUp() override {
    if (state_ == RC)
      arm_servo_sender_->setAngularVel(data_.dbus_data_.ch_l_x, data_.dbus_data_.ch_l_y, data_.dbus_data_.ch_r_y);
  }
  void remoteControlTurnOn() override {
    std_srvs::Empty srv;
    ManualBase::remoteControlTurnOn();
    reset_servo_server_.call(srv);
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
  ros::Publisher pub_;
  rm_common::Vel3DCommandSender *arm_servo_sender_{};
  ros::ServiceClient reset_servo_server_;
  std::string target_frame_, source_frame_;
  bool has_send_step_list_{};
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
