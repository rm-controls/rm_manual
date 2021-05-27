//
// Created by qiayuan on 5/23/21.
//

#ifndef RM_MANUAL_ENGINEER_MANUAL_H_
#define RM_MANUAL_ENGINEER_MANUAL_H_
#include <std_srvs/Empty.h>
#include "rm_manual/chassis_gimbal_manual.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_manual {
class EngineerManual : public ChassisGimbalManual {
 public:
  explicit EngineerManual(ros::NodeHandle &nh) : ChassisGimbalManual(nh) {
    ros::NodeHandle arm_servo(nh, "arm_servo");
    arm_servo_sender_ = new Vel3DCommandSender(arm_servo);
    reset_servo_server_ =
        nh.serviceClient<std_srvs::Empty>("/servo_server/reset_servo_status");
    if (!nh.getParam("target_frame", target_frame_))
      ROS_ERROR("Target frame no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!nh.getParam("source_frame", source_frame_))
      ROS_ERROR("Source frame no defined (namespace: %s)", nh.getNamespace().c_str());
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
    if (state_ == RC) {
      geometry_msgs::Vector3 scale;   // velocity under base_link frame
      scale.x = data_.dbus_data_.ch_r_x;
      scale.y = -data_.dbus_data_.ch_r_y;
      scale.z = data_.dbus_data_.ch_l_y;
/*      //TODO: Add frame names to params server
      try { tf2::doTransform(scale, scale, data_.tf_buffer_.lookupTransform("link5", "base_link", ros::Time(0))); }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
      }*/
      arm_servo_sender_->setLinearVel(scale.x, scale.y, scale.z);
    }
  }
  void leftSwitchUp() override {
    if (state_ == RC) {
      arm_servo_sender_->setAngularVel(data_.dbus_data_.ch_l_x, data_.dbus_data_.ch_l_y, data_.dbus_data_.ch_r_y);
    }
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
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
