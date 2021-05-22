//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_

#include "rm_manual/common/manual_base.h"
namespace rm_manual {
class ChassisGimbalManual : public ManualBase {
 public:
  explicit ChassisGimbalManual(ros::NodeHandle &nh) : ManualBase(nh) {
    ros::NodeHandle chassis_nh(nh, "chassis");
    chassis_cmd_sender_ = new ChassisCommandSender(chassis_nh, *data_.referee_);
    ros::NodeHandle vel_nh(nh, "vel");
    vel_cmd_sender_ = new VelCommandSender(vel_nh);
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new GimbalCommandSender(gimbal_nh, *data_.referee_);
  }
 protected:
  void rightSwitchMid() override {
    ManualBase::rightSwitchMid();
    if (std::abs(data_.dbus_data_.wheel) > 0.01) {
      vel_cmd_sender_->setWVel(data_.dbus_data_.wheel);
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    } else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setXVel(data_.dbus_data_.ch_r_y);
    vel_cmd_sender_->setYVel(data_.dbus_data_.ch_r_x);
  }
  void rightSwitchDown() override {
    ManualBase::rightSwitchDown();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::PASSIVE);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::PASSIVE);
  }
  void leftSwitchDown() override {
    ManualBase::leftSwitchDown();
    if (state_ == RC) {
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
      gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
    }
  }
  void leftSwitchMid() override {
    ManualBase::leftSwitchMid();
    if (state_ == RC) {
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->updateCost(data_.track_data_array_);
    }
  }
  void leftSwitchUp() override {
    ManualBase::leftSwitchUp();
    if (state_ == RC) {
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->updateCost(data_.track_data_array_);
    }
  }
  void sendCommand(const ros::Time &time) override {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
  }
  void wPress() override { if (state_ == PC) vel_cmd_sender_->setXVel(1.); }
  void aPress() override { if (state_ == PC) vel_cmd_sender_->setYVel(1.); }
  void sPress() override { if (state_ == PC) vel_cmd_sender_->setXVel(-1.); }
  void dPress() override { if (state_ == PC) vel_cmd_sender_->setYVel(-1.); }
  ChassisCommandSender *chassis_cmd_sender_;
  VelCommandSender *vel_cmd_sender_;
  GimbalCommandSender *gimbal_cmd_sender_;
  bool have_power_manager_{};
  double safety_power_{};
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
