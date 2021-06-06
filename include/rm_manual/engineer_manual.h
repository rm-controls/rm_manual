//
// Created by qiayuan on 5/23/21.
//

#ifndef RM_MANUAL_ENGINEER_MANUAL_H_
#define RM_MANUAL_ENGINEER_MANUAL_H_
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>

#include <rm_msgs/EngineerAction.h>
#include "rm_manual/chassis_gimbal_manual.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_manual {
class EngineerManual : public ChassisGimbalManual {
 public:
  explicit EngineerManual(ros::NodeHandle &nh)
      : ChassisGimbalManual(nh), action_client_("/engineer_middleware/move_arm", true) {
    ros::NodeHandle arm_servo(nh, "arm_servo");
    arm_servo_sender_ = new Vel3DCommandSender(arm_servo);
    reset_servo_server_ =
        nh.serviceClient<std_srvs::Empty>("/servo_server/reset_servo_status");
    ROS_INFO("Waiting for move arm server to start.");
    // wait for the action server to start
    action_client_.waitForServer(); //will wait for infinite time
    ROS_INFO("Move arm server started.");
  }
 private:
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    arm_servo_sender_->sendCommand(time);
  }
  void setZero() override {
    ChassisGimbalManual::setZero();
    arm_servo_sender_->setZero();
  }
  void rightSwitchMid() override {
    if (data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN)
      ChassisGimbalManual::rightSwitchMid();
  }
  void leftSwitchMid() override {
    rm_msgs::EngineerActionGoal goal;
    goal.goal.step = "grasp_small_resource";
    if (state_ == RC) {
      if (action_client_.isServerConnected()) {
        action_client_.sendGoal(goal.goal);
        if (action_client_.waitForResult(ros::Duration(30.0)))
          ROS_INFO("finish grasp_small_resource");
      } else
        ROS_WARN("Can not connected with move arm server");
    }

  }
  void leftSwitchUp() override {
    if (state_ == RC)
      arm_servo_sender_->setAngularVel(data_.dbus_data_.ch_l_x, data_.dbus_data_.ch_l_y, data_.dbus_data_.ch_r_y);
  }
  void remoteControlTurnOn() override {
    std_srvs::Empty srv;
    ManualBase::remoteControlTurnOn();
    reset_servo_server_.call(srv);
    ROS_INFO("reset arm");
  }
  Vel3DCommandSender *arm_servo_sender_{};
  ros::ServiceClient reset_servo_server_;
  std::string target_frame_, source_frame_;
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
