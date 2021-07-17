//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#include "rm_manual/chassis_gimbal_manual.h"

namespace rm_manual {
class ChassisGimbalShooterManual : public ChassisGimbalManual {
 public:
  explicit ChassisGimbalShooterManual(ros::NodeHandle &nh) : ChassisGimbalManual(nh) {
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh);
    ros::NodeHandle cover_nh(nh, "cover");
    cover_command_sender_ = new rm_common::CoverCommandSender(cover_nh);
    ui_shooter_ = new UiShooter(&data_.referee_);
    ros::NodeHandle enemy_color_nh(nh, "enemy_color_switch");
    switch_enemy_color_srv_ = new rm_common::SwitchEnemyColorService(enemy_color_nh);
    ros::NodeHandle target_type_nh(nh, "target_type_switch");
    switch_target_type_srv_ = new rm_common::SwitchTargetTypeService(target_type_nh);
    ui_capacitor_ = new UiCapacitor(&data_.referee_);
    ui_target_ = new UiTarget(&data_.referee_);
    ui_cover_ = new UiCover(&data_.referee_);
  }
  void run() override {
    ManualBase::run();
    switch_enemy_color_srv_->setEnemyColor(data_.referee_.referee_data_);
  }
 protected:
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
    cover_command_sender_->sendCommand(time);
  }
  void shooterOutputOn() override {
    ROS_INFO("Shooter Output on!");
    calibration_manager_->reset();
  }
  void rightSwitchDown() override {
    ChassisGimbalManual::rightSwitchDown();
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    cover_command_sender_->open();
  }
  void rightSwitchMid() override {
    ChassisGimbalManual::rightSwitchMid();
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    cover_command_sender_->close();
  }
  void rightSwitchUp() override {
    ChassisGimbalManual::rightSwitchUp();
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    cover_command_sender_->close();
    ui_shooter_->setOperateType(UPDATE);
    ui_capacitor_->setOperateType(UPDATE);
    ui_target_->setOperateType(UPDATE);
  }
  void leftSwitchDown() override {
    ChassisGimbalManual::leftSwitchDown();
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void leftSwitchMid() override {
    ChassisGimbalManual::leftSwitchMid();
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->updateCost(data_.referee_.referee_data_, data_.track_data_array_);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    shooter_cmd_sender_->updateLimit(data_.referee_.referee_data_);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
  void leftSwitchUp() override {
    ChassisGimbalManual::leftSwitchUp();
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->updateCost(data_.referee_.referee_data_, data_.track_data_array_);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    shooter_cmd_sender_->updateLimit(data_.referee_.referee_data_);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
  }
  void fPress() override { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void qPress() override { shooter_cmd_sender_->setBurstMode(!shooter_cmd_sender_->getBurstMode()); }
  void xPress() override {
    ChassisGimbalManual::xPress();
    ui_shooter_->setOperateType(ADD);
    ui_capacitor_->setOperateType(ADD);
    ui_target_->setOperateType(ADD);
  }
  void xRelease() override {
    ChassisGimbalManual::xRelease();
    ui_shooter_->setOperateType(UPDATE);
    ui_capacitor_->setOperateType(UPDATE);
    ui_target_->setOperateType(UPDATE);
  }
  void mouseLeftPress() override {
    shooter_cmd_sender_->updateLimit(data_.referee_.referee_data_);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  }
  void mouseLeftRelease() override { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY); }
  void mouseRightPress() override {
    if (cover_command_sender_->isClose()) {
      gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->updateCost(data_.referee_.referee_data_, data_.track_data_array_);
      shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
    }
  }
  void mouseRightRelease() override {
    if (cover_command_sender_->isClose())
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  void ctrlRPress() override {
    switch_target_type_srv_->switchTargetType();
    switch_target_type_srv_->callService();
    if (switch_target_type_srv_->getTarget() == "buff")
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  void ctrlVPress() override {
    switch_enemy_color_srv_->switchEnemyColor();
    switch_enemy_color_srv_->callService();
  }
  void ctrlCPress() override {
    gimbal_cmd_sender_->setBaseOnly(!gimbal_cmd_sender_->getBaseOnly());
  }
  void ctrlZPress() override {
    if (data_.referee_.robot_id_ != RobotId::BLUE_HERO && data_.referee_.robot_id_ != RobotId::RED_HERO) {
      if (cover_command_sender_->isClose()) {
        geometry_msgs::PointStamped aim_point{};
        aim_point.header.frame_id = "yaw";
        aim_point.header.stamp = ros::Time::now();
        aim_point.point.x = 1000;
        aim_point.point.y = 0;
        aim_point.point.z = 0;
        gimbal_cmd_sender_->setAimPoint(aim_point);
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
        cover_command_sender_->open();
      } else {
        gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        cover_command_sender_->close();
      }
    }
  }
  void drawUi() override {
    ChassisGimbalManual::drawUi();
    ros::Time time = ros::Time::now();
    ui_shooter_->display(time, shooter_cmd_sender_->getMsg()->mode, shooter_cmd_sender_->getBurstMode());
    ui_target_->display(time, switch_target_type_srv_->getTarget(), switch_enemy_color_srv_->getColor(),
                        gimbal_cmd_sender_->getBaseOnly());
    ui_capacitor_->display(time);
    ui_cover_->display(cover_command_sender_->isClose());
  }
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::CoverCommandSender *cover_command_sender_{};
  UiShooter *ui_shooter_{};
  UiCapacitor *ui_capacitor_{};
  UiTarget *ui_target_{};
  UiCover *ui_cover_{};
  rm_common::SwitchEnemyColorService *switch_enemy_color_srv_{};
  rm_common::SwitchTargetTypeService *switch_target_type_srv_{};
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
