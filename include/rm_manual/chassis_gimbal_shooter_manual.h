//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
#include "rm_manual/chassis_gimbal_manual.h"
#include <rm_common/decision/calibration_queue.h>

namespace rm_manual {
class ChassisGimbalShooterManual : public ChassisGimbalManual {
 public:
  explicit ChassisGimbalShooterManual(ros::NodeHandle &nh) :
      ChassisGimbalManual(nh),
      q_press_event_(boost::bind(&ChassisGimbalShooterManual::qPress, this, _1)),
      f_press_event_(boost::bind(&ChassisGimbalShooterManual::fPress, this, _1)),
      shift_press_event_(boost::bind(&ChassisGimbalShooterManual::shiftPress, this, _1)),
      shift_release_event_(boost::bind(&ChassisGimbalShooterManual::shiftRelease, this, _1)),
      ctrl_z_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlZPress, this, _1)),
      ctrl_c_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlCPress, this, _1)),
      ctrl_v_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlVPress, this, _1)),
      ctrl_r_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlRPress, this, _1)) {
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_.referee_.referee_data_);
    ros::NodeHandle cover_nh(nh, "cover");
    cover_command_sender_ = new rm_common::CoverCommandSender(cover_nh);
    ui_shooter_ = new UiShooter(&data_.referee_);
    ros::NodeHandle enemy_color_nh(nh, "enemy_color_switch");
    switch_enemy_color_srv_ = new rm_common::SwitchEnemyColorServiceCaller(enemy_color_nh);
    ros::NodeHandle target_type_nh(nh, "target_type_switch");
    switch_target_type_srv_ = new rm_common::SwitchTargetTypeServiceCaller(target_type_nh);

    XmlRpc::XmlRpcValue rpc_value;
    nh.getParam("shooter_calibration", rpc_value);
    shooter_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
    ui_target_ = new UiTarget(&data_.referee_);
    ui_cover_ = new UiCover(&data_.referee_);
  }
  void run() override {
    ManualBase::run();
    switch_enemy_color_srv_->setEnemyColor(data_.referee_.referee_data_);
//    shooter_calibration_->update(ros::Time::now());
  }
 protected:
  void checkKeyboard() override {
    ManualBase::checkKeyboard();
    q_press_event_.update(data_.dbus_data_.key_q);
    f_press_event_.update(data_.dbus_data_.key_f);
    shift_press_event_.update(data_.dbus_data_.key_shift);
    shift_release_event_.update(data_.dbus_data_.key_shift);
    ctrl_z_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z);
    ctrl_c_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
    ctrl_v_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v);
    ctrl_r_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  }
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
    cover_command_sender_->sendCommand(time);
  }
  void shooterOutputOn(ros::Duration /*duration*/) override {
    ROS_INFO("Shooter Output on!");
//    shooter_calibration_->reset();
  }
  void updateRc() override {
    ChassisGimbalManual::updateRc();
    gimbal_cmd_sender_->updateCost(data_.track_data_array_);
  }
  void rightSwitchDown(ros::Duration duration) override {
    ChassisGimbalManual::rightSwitchDown(duration);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    cover_command_sender_->open();
  }
  void rightSwitchMid(ros::Duration duration) override {
    ChassisGimbalManual::rightSwitchMid(duration);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    cover_command_sender_->close();
  }
  void rightSwitchUp(ros::Duration duration) override {
    ChassisGimbalManual::rightSwitchUp(duration);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    cover_command_sender_->close();
    ui_shooter_->setOperateType(UPDATE);
    ui_capacitor_->setOperateType(UPDATE);
    ui_target_->setOperateType(UPDATE);
  }
  void leftSwitchDown(ros::Duration duration) override {
    ChassisGimbalManual::leftSwitchDown(duration);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void leftSwitchMid(ros::Duration duration) override {
    ChassisGimbalManual::leftSwitchMid(duration);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
  void leftSwitchUp(ros::Duration duration) override {
    ChassisGimbalManual::leftSwitchUp(duration);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
  }
  void xPress(ros::Duration duration) override {
    ChassisGimbalManual::xPress(duration);
    ui_shooter_->setOperateType(ADD);
    ui_target_->setOperateType(ADD);
  }
  void xRelease(ros::Duration duration) override {
    ChassisGimbalManual::xRelease(duration);
    ui_shooter_->setOperateType(UPDATE);
    ui_target_->setOperateType(UPDATE);
  }
  void mouseLeftPress(ros::Duration /*duration*/) override {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
  }
  void mouseLeftRelease(ros::Duration /*duration*/) override { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY); }
  void mouseRightPress(ros::Duration /*duration*/) override {
    if (cover_command_sender_->isClose()) {
      gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->updateCost(data_.track_data_array_);
      shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
    }
  }
  void mouseRightRelease(ros::Duration /*duration*/) override {
    if (cover_command_sender_->isClose())
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  void fPress(ros::Duration /*duration*/) { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void qPress(ros::Duration /*duration*/) { shooter_cmd_sender_->setBurstMode(!shooter_cmd_sender_->getBurstMode()); }
  void shiftPress(ros::Duration /*duration*/) { chassis_cmd_sender_->setBurstMode(true); }
  void shiftRelease(ros::Duration /*duration*/) { chassis_cmd_sender_->setBurstMode(false); }
  void ctrlRPress(ros::Duration /*duration*/) {
    switch_target_type_srv_->switchTargetType();
    switch_target_type_srv_->callService();
    if (switch_target_type_srv_->getTarget() == "buff")
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  void ctrlVPress(ros::Duration /*duration*/) {
    switch_enemy_color_srv_->switchEnemyColor();
    switch_enemy_color_srv_->callService();
  }
  void ctrlCPress(ros::Duration /*duration*/) {
    gimbal_cmd_sender_->setBaseOnly(!gimbal_cmd_sender_->getBaseOnly());
  }
  void ctrlZPress(ros::Duration /*duration*/) {
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
    ui_cover_->display(cover_command_sender_->isClose());
  }
  RisingInputEvent q_press_event_;
  RisingInputEvent f_press_event_;
  RisingInputEvent shift_press_event_;
  FallingInputEvent shift_release_event_;
  RisingInputEvent ctrl_z_press_event_;
  RisingInputEvent ctrl_c_press_event_;
  RisingInputEvent ctrl_v_press_event_;
  RisingInputEvent ctrl_r_press_event_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::CoverCommandSender *cover_command_sender_{};
  rm_common::SwitchEnemyColorServiceCaller *switch_enemy_color_srv_{};
  rm_common::SwitchTargetTypeServiceCaller *switch_target_type_srv_{};
  rm_common::CalibrationQueue *shooter_calibration_;
  UiShooter *ui_shooter_{};
  UiTarget *ui_target_{};
  UiCover *ui_cover_{};
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
