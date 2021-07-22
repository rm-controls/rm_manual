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
      ctrl_c_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlCPress, this, _1)),
      ctrl_v_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlVPress, this, _1)),
      ctrl_r_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlRPress, this, _1)),
      ctrl_b_press_event_(boost::bind(&ChassisGimbalShooterManual::ctrlBPress, this, _1)) {
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh, data_.referee_.referee_data_);
    ui_shooter_ = new UiShooter(&data_.referee_);
    ros::NodeHandle detection_switch_nh(nh, "detection_switch");
    switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);

    XmlRpc::XmlRpcValue rpc_value;
    nh.getParam("trigger_calibration", rpc_value);
    trigger_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
    ui_target_ = new UiTarget(&data_.referee_);
    ui_cover_ = new UiCover(&data_.referee_);
  }
  void run() override {
    ChassisGimbalManual::run();
    switch_detection_srv_->setEnemyColor(data_.referee_.referee_data_);
    trigger_calibration_->update(ros::Time::now());
  }
 protected:
  void checkKeyboard() override {
    ChassisGimbalManual::checkKeyboard();
    q_press_event_.update(data_.dbus_data_.key_q);
    f_press_event_.update(data_.dbus_data_.key_f);
    shift_press_event_.update(data_.dbus_data_.key_shift);
    shift_release_event_.update(data_.dbus_data_.key_shift);
    ctrl_c_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
    ctrl_v_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v);
    ctrl_r_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
    ctrl_b_press_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_b);
  }
  void sendCommand(const ros::Time &time) override {
    ChassisGimbalManual::sendCommand(time);
    shooter_cmd_sender_->sendCommand(time);
  }
  void shooterOutputOn(ros::Duration /*duration*/) override {
    ROS_INFO("Shooter Output ON");
    trigger_calibration_->reset();
  }
  void updateRc() override {
    ChassisGimbalManual::updateRc();
    if (shooter_cmd_sender_->getMsg()->mode != rm_msgs::ShootCmd::STOP) {
      gimbal_cmd_sender_->updateCost(data_.track_data_array_);
      gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    }
  }
  void rightSwitchDown(ros::Duration duration) override {
    ChassisGimbalManual::rightSwitchDown(duration);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void rightSwitchMid(ros::Duration duration) override {
    ChassisGimbalManual::rightSwitchMid(duration);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void rightSwitchUp(ros::Duration duration) override {
    ChassisGimbalManual::rightSwitchUp(duration);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    ui_shooter_->setOperateType(UPDATE);
    ui_capacitor_->setOperateType(UPDATE);
    ui_target_->setOperateType(UPDATE);
  }
  void leftSwitchDown(ros::Duration duration) override {
    ChassisGimbalManual::leftSwitchDown(duration);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void leftSwitchMid(ros::Duration duration) override {
    ChassisGimbalManual::leftSwitchMid(duration);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
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
    gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_cmd_sender_->updateCost(data_.track_data_array_);
    shooter_cmd_sender_->checkError(data_.gimbal_des_error_, ros::Time::now());
  }
  void mouseRightRelease(ros::Duration /*duration*/) override {
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  void fPress(ros::Duration /*duration*/) { shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP); }
  void qPress(ros::Duration /*duration*/) { shooter_cmd_sender_->setBurstMode(!shooter_cmd_sender_->getBurstMode()); }
  void shiftPress(ros::Duration /*duration*/) { chassis_cmd_sender_->setBurstMode(true); }
  void shiftRelease(ros::Duration /*duration*/) { chassis_cmd_sender_->setBurstMode(false); }
  void ctrlCPress(ros::Duration /*duration*/) {
    gimbal_cmd_sender_->setBaseOnly(!gimbal_cmd_sender_->getBaseOnly());
  }
  void ctrlVPress(ros::Duration /*duration*/) {
    switch_detection_srv_->switchEnemyColor();
    switch_detection_srv_->callService();
  }
  void ctrlRPress(ros::Duration /*duration*/) {
    switch_detection_srv_->switchTargetType();
    switch_detection_srv_->callService();
    if (switch_detection_srv_->getTarget())
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
    else
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  void ctrlBPress(ros::Duration /*duration*/) {
    switch_detection_srv_->switchExposureLevel();
    switch_detection_srv_->callService();
  }
  void drawUi() override {
    ChassisGimbalManual::drawUi();
    ros::Time time = ros::Time::now();
    std::string enemy_color =
        switch_detection_srv_->getColor() == rm_msgs::StatusChangeRequest::BLUE ? "blue" : "red";
    std::string target_type =
        switch_detection_srv_->getTarget() == rm_msgs::StatusChangeRequest::BUFF ? "buff" : "armor";
    ui_shooter_->display(time, shooter_cmd_sender_->getMsg()->mode, shooter_cmd_sender_->getBurstMode());
    ui_target_->display(time, target_type, enemy_color, gimbal_cmd_sender_->getBaseOnly());
  }
  RisingInputEvent q_press_event_;
  RisingInputEvent f_press_event_;
  RisingInputEvent shift_press_event_;
  FallingInputEvent shift_release_event_;
  RisingInputEvent ctrl_c_press_event_;
  RisingInputEvent ctrl_v_press_event_;
  RisingInputEvent ctrl_r_press_event_;
  RisingInputEvent ctrl_b_press_event_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_{};
  rm_common::SwitchDetectionCaller *switch_detection_srv_{};
  rm_common::CalibrationQueue *trigger_calibration_;
  UiShooter *ui_shooter_{};
  UiTarget *ui_target_{};
  UiCover *ui_cover_{};
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_SHOOTER_MANUAL_H_
