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
    chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh, data_.referee_.referee_data_);
    ros::NodeHandle vel_nh(nh, "vel");
    vel_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh, data_.referee_.referee_data_);
    ros::NodeHandle ui_nh(nh, "ui");
    title_ui_ = new TitleUi(ui_nh, data_.referee_);
    state_ui_ = new StateUi(ui_nh, data_.referee_);
    warning_ui_ = new WarningUi(ui_nh, data_.referee_);
  }
 protected:
  void sendCommand(const ros::Time &time) override {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
  }
  void updateRc() override {
    ManualBase::updateRc();
    if (std::abs(data_.dbus_data_.wheel) > 0.01) {
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    } else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setAngularZVel(data_.dbus_data_.wheel);
    vel_cmd_sender_->setLinearXVel(data_.dbus_data_.ch_r_y);
    vel_cmd_sender_->setLinearYVel(-data_.dbus_data_.ch_r_x);
    gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
  }
  void updatePc() override {
    ManualBase::updatePc();
    gimbal_cmd_sender_->setRate(-data_.dbus_data_.m_x, data_.dbus_data_.m_y);
  }
  void rightSwitchDown(ros::Duration duration) override {
    ManualBase::rightSwitchDown(duration);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setZero();
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    gimbal_cmd_sender_->setZero();
  }
  void rightSwitchMid(ros::Duration duration) override {
    ManualBase::rightSwitchMid(duration);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  void rightSwitchUp(ros::Duration duration) override {
    ManualBase::rightSwitchUp(duration);
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setZero();
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  void leftSwitchDown(ros::Duration duration) override {
    ManualBase::leftSwitchDown(duration);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  void wPress(ros::Duration /*duration*/) override { vel_cmd_sender_->setLinearXVel(1.); }
  void wRelease(ros::Duration /*duration*/) override { vel_cmd_sender_->setLinearXVel(0.); }
  void aPress(ros::Duration /*duration*/) override { vel_cmd_sender_->setLinearYVel(1.); }
  void aRelease(ros::Duration /*duration*/) override { vel_cmd_sender_->setLinearYVel(0.); }
  void sPress(ros::Duration /*duration*/) override { vel_cmd_sender_->setLinearXVel(-1.); }
  void sRelease(ros::Duration /*duration*/) override { vel_cmd_sender_->setLinearXVel(0.); }
  void dPress(ros::Duration /*duration*/) override { vel_cmd_sender_->setLinearYVel(-1.); }
  void dRelease(ros::Duration /*duration*/) override { vel_cmd_sender_->setLinearYVel(0.); }
  void xPress(ros::Duration /*duration*/) override {
    title_ui_->add();
    state_ui_->add();
  }
  void xRelease(ros::Duration /*duration*/) override {}
  void gPress(ros::Duration /*duration*/) override {
    if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::GYRO) {
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
      vel_cmd_sender_->setAngularZVel(0.);
    } else {
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
      vel_cmd_sender_->setAngularZVel(1.);
    }
  }
  void ePress(ros::Duration /*duration*/) override {
    if (chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::TWIST)
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::TWIST);
  }
  void drawUi() override {
    state_ui_->update("chassis", chassis_cmd_sender_->getMsg()->mode, chassis_cmd_sender_->getBurstMode());
    state_ui_->update("gimbal", gimbal_cmd_sender_->getMsg()->mode);
    warning_ui_->update("spin", chassis_cmd_sender_->getMsg()->mode != rm_msgs::ChassisCmd::GYRO, ros::Time::now());
  }
  rm_common::ChassisCommandSender *chassis_cmd_sender_{};
  rm_common::Vel2DCommandSender *vel_cmd_sender_;
  rm_common::GimbalCommandSender *gimbal_cmd_sender_{};
  TitleUi *title_ui_{};
  StateUi *state_ui_{};
  WarningUi *warning_ui_{};
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
