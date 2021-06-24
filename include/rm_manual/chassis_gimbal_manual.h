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
    chassis_cmd_sender_ = new ChassisCommandSender(chassis_nh, data_.referee_);
    ros::NodeHandle vel_nh(nh, "vel");
    vel_cmd_sender_ = new Vel2DCommandSender(vel_nh);
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new GimbalCommandSender(gimbal_nh, data_.referee_);
  }
 protected:
  void drawUi() override {
    if (state_ == PC) {
      ui_->displayCapInfo();
      ui_->displayArmorInfo(ros::Time::now());
      ui_->displayChassisInfo(chassis_cmd_sender_->getMsg()->mode, data_.dbus_data_.key_shift);
      ui_->displayGimbalInfo(chassis_cmd_sender_->getMsg()->mode);
    }
  }
  void sendCommand(const ros::Time &time) override {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
  }
  void setZero() override {
    chassis_cmd_sender_->setZero();
    vel_cmd_sender_->setZero();
    gimbal_cmd_sender_->setZero();
  }
  void rightSwitchMid() override {
    ManualBase::rightSwitchMid();
    if (std::abs(data_.dbus_data_.wheel) > 0.01) {
      vel_cmd_sender_->setAngularZVel(data_.dbus_data_.wheel);
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    } else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setLinearXVel(data_.dbus_data_.ch_r_y);
    vel_cmd_sender_->setLinearYVel(-data_.dbus_data_.ch_r_x);
    gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
  }
  void rightSwitchUp() override {
    ManualBase::rightSwitchUp();
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x, data_.dbus_data_.m_y);
    if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO)
      vel_cmd_sender_->setAngularZVel(1.);
    ui_->setOperateType(UPDATE);
  }
  void leftSwitchDown() override {
    ManualBase::leftSwitchDown();
    if (state_ == RC) { gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE); }
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
  void wPress() override { if (state_ == PC) vel_cmd_sender_->setLinearXVel(1.); }
  void aPress() override { if (state_ == PC) vel_cmd_sender_->setLinearYVel(1.); }
  void sPress() override { if (state_ == PC) vel_cmd_sender_->setLinearXVel(-1.); }
  void dPress() override { if (state_ == PC) vel_cmd_sender_->setLinearYVel(-1.); }
  void xPress() override { if (state_ == PC) ui_->setOperateType(ADD); }
  void gPress() override {
    if (state_ == PC && ros::Time::now() - last_release_g_ < ros::Duration(0.015)) {
      if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO)
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
      else chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    }
  }
  void ePress() override {
    if (state_ == PC && ros::Time::now() - last_release_e_ < ros::Duration(0.015)) {
      if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::TWIST)
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
      else if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::TWIST);
    }
  }
  void mouseRightPress() override {
    if (state_ == PC) {
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->updateCost(data_.track_data_array_);
    }
  }
  void ctrlZPress() override {
    if (state_ == PC)
      state_ = IDLE;
  }
  void ctrlWPress() override {
    if (state_ == IDLE)
      state_ = PC;
  }
  ChassisCommandSender *chassis_cmd_sender_{};
  Vel2DCommandSender *vel_cmd_sender_;
  GimbalCommandSender *gimbal_cmd_sender_{};
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
