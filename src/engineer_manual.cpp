//
// Created by qiayuan on 7/25/21.
//

#include "rm_manual/engineer_manual.h"

namespace rm_manual {
EngineerManual::EngineerManual(ros::NodeHandle &nh)
    : ChassisGimbalManual(nh), operating_mode_(MANUAL), action_client_("/engineer_middleware/move_arm", true) {
  ROS_INFO("Waiting for middleware to start.");
  action_client_.waitForServer();
  ROS_INFO("Middleware started.");
  // Command sender
  ros::NodeHandle nh_card(nh, "card");
  card_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_card);
  ros::NodeHandle nh_mast(nh, "mast");
  mast_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_mast);
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("power_on_calibration", rpc_value);
  power_on_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("arm_calibration", rpc_value);
  arm_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  left_switch_up_event_.setFalling(boost::bind(&EngineerManual::leftSwitchUpFall, this));
  left_switch_down_event_.setFalling(boost::bind(&EngineerManual::leftSwitchDownFall, this));
  ctrl_c_event_.setRising(boost::bind(&EngineerManual::ctrlCPress, this));
  ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));
  ctrl_w_event_.setRising(boost::bind(&EngineerManual::ctrlWPress, this));
  ctrl_s_event_.setRising(boost::bind(&EngineerManual::ctrlSPress, this));
  ctrl_q_event_.setRising(boost::bind(&EngineerManual::ctrlQPress, this));
}

void EngineerManual::run() {
  ChassisGimbalManual::run();
  arm_calibration_->update(ros::Time::now());
  power_on_calibration_->update(ros::Time::now());
}

void EngineerManual::checkKeyboard() {
  ChassisGimbalManual::checkKeyboard();
  ctrl_c_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
  ctrl_f_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_f);
  ctrl_r_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  ctrl_w_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_w);
  ctrl_s_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_s);
  ctrl_q_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_q);
}

void EngineerManual::updateRc() {
  ChassisGimbalManual::updateRc();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  left_switch_up_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN);
}

void EngineerManual::updatePc() {
  ChassisGimbalManual::updatePc();
  vel_cmd_sender_->setAngularZVel(-data_.dbus_data_.m_x);
}

void EngineerManual::sendCommand(const ros::Time &time) {
  mast_command_sender_->sendCommand(time);
  if (operating_mode_ == MANUAL) {
    ChassisGimbalManual::sendCommand(time);
    card_command_sender_->sendCommand(time);
  }
}

void EngineerManual::drawUi(const ros::Time &time) {
  ChassisGimbalManual::drawUi(time);
  time_change_ui_->update("effort", time);
  flash_ui_->update("calibration", time, power_on_calibration_->isCalibrated());
//    trigger_change_ui_->update("jog", jog_joint_name);
}

void EngineerManual::remoteControlTurnOff() {
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}

void EngineerManual::chassisOutputOn() {
  power_on_calibration_->reset();
  if (MIDDLEWARE)
    action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchDownRise() {
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchMidRise() {
  ChassisGimbalManual::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::rightSwitchUpRise() {
  ChassisGimbalManual::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::runStepQueue(std::string step_queue_name) {
  rm_msgs::EngineerGoal goal;
  goal.step_queue_name = std::move(step_queue_name);
  if (action_client_.isServerConnected()) {
    if (operating_mode_ == MANUAL)
      action_client_.sendGoal(goal,
                              boost::bind(&EngineerManual::actionDoneCallback, this, _1, _2),
                              boost::bind(&EngineerManual::actionActiveCallback, this),
                              boost::bind(&EngineerManual::actionFeedbackCb, this, _1));
    operating_mode_ = MIDDLEWARE;
  } else
    ROS_ERROR("Can not connect to middleware");
}

void EngineerManual::actionFeedbackCb(const rm_msgs::EngineerFeedbackConstPtr &feedback) {
  trigger_change_ui_->update("queue", feedback->current_step);
  time_change_ui_->update("progress", ros::Time::now(), ((double) feedback->finished_step) / feedback->total_steps);
}

void EngineerManual::actionDoneCallback(const actionlib::SimpleClientGoalState &state,
                                        const rm_msgs::EngineerResultConstPtr &result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->finish);
  operating_mode_ = MANUAL;
}

}