//
// Created by qiayuan on 5/23/21.
//

#ifndef RM_MANUAL_ENGINEER_MANUAL_H_
#define RM_MANUAL_ENGINEER_MANUAL_H_

#include "rm_manual/chassis_gimbal_manual.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_manual {
class EngineerManual : public ChassisGimbalManual {
 public:
  explicit EngineerManual(ros::NodeHandle &nh) : ChassisGimbalManual(nh) {
    ros::NodeHandle arm_servo(nh, "arm_servo");
    arm_servo_sender_ = new Vel3DCommandSender(arm_servo);
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
      scale.x = data_.dbus_data_.ch_r_y;
      scale.y = -data_.dbus_data_.ch_r_y;
      scale.z = data_.dbus_data_.ch_l_y;
      //TODO: Add frame names to params server
      try { tf2::doTransform(scale, scale, data_.tf_buffer_.lookupTransform("link5", "base_link", ros::Time(0))); }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
      }
      arm_servo_sender_->setLinearVel(scale.x, scale.y, scale.z);
    }
  }
  void leftSwitchUp() override {
  }

  Vel3DCommandSender *arm_servo_sender_{};
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
