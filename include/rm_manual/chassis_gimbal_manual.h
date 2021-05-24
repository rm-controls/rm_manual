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
  void drawUi() override {
    referee_ui_->displayCapInfo(graphic_operate_type_);
    referee_ui_->displayArmorInfo(ros::Time::now());
    graphic_operate_type_ = UPDATE;
  }
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
  void rightSwitchUp() override {
    ManualBase::rightSwitchUp();
    gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x, data_.dbus_data_.m_y);
    gimbal_cmd_sender_->setMode(pc_gimbal_mode_);
    chassis_cmd_sender_->setMode(pc_chassis_mode_);
    vel_cmd_sender_->setVel(0., 0., 1.);
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
  void wPress() override { if (state_ == PC) vel_cmd_sender_->setXVel(1.); }
  void aPress() override { if (state_ == PC) vel_cmd_sender_->setYVel(1.); }
  void sPress() override { if (state_ == PC) vel_cmd_sender_->setXVel(-1.); }
  void dPress() override { if (state_ == PC) vel_cmd_sender_->setYVel(-1.); }
  void xPress() override { if (state_ == PC) graphic_operate_type_ = ADD; }
  void ePress() override {
    if (state_ == PC && ros::Time::now() - last_release_e_ < ros::Duration(0.1)) {
      pc_chassis_mode_ =
          (pc_chassis_mode_ == rm_msgs::ChassisCmd::GYRO) ? rm_msgs::ChassisCmd::FOLLOW : rm_msgs::ChassisCmd::GYRO;
      referee_ui_->displayChassisInfo(pc_chassis_mode_, chassis_burst_flag_, graphic_operate_type_);
    }
  }
  void rPress() override {
    if (state_ == PC && ros::Time::now() - last_release_r_ < ros::Duration(0.1)) {
      pc_chassis_mode_ =
          (pc_chassis_mode_ == rm_msgs::ChassisCmd::TWIST) ? rm_msgs::ChassisCmd::FOLLOW : rm_msgs::ChassisCmd::TWIST;
      referee_ui_->displayChassisInfo(pc_chassis_mode_, chassis_burst_flag_, graphic_operate_type_);
    }
  }
  void mouseRightPress() override {
    if (state_ == PC) {
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->updateCost(data_.track_data_array_);
    }
  }
  void ctrlZPress() override {
    if (state_ == PC) {
      pc_chassis_mode_ = rm_msgs::ChassisCmd::PASSIVE;
      pc_gimbal_mode_ = rm_msgs::GimbalCmd::PASSIVE;
      referee_ui_->displayChassisInfo(pc_chassis_mode_, chassis_burst_flag_, graphic_operate_type_);
      referee_ui_->displayGimbalInfo(pc_gimbal_mode_, graphic_operate_type_);
    }
  }
  void ctrlWPress() override {
    if (state_ == PC) {
      pc_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
      pc_gimbal_mode_ = rm_msgs::GimbalCmd::RATE;
      referee_ui_->displayChassisInfo(pc_chassis_mode_, chassis_burst_flag_, graphic_operate_type_);
      referee_ui_->displayGimbalInfo(pc_gimbal_mode_, graphic_operate_type_);
    }
  }
  void sendCommand(const ros::Time &time) override {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
  }
  ChassisCommandSender *chassis_cmd_sender_;
  VelCommandSender *vel_cmd_sender_;
  GimbalCommandSender *gimbal_cmd_sender_;
  int pc_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW, pc_gimbal_mode_ = rm_msgs::GimbalCmd::RATE;
  bool chassis_burst_flag_ = false;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
