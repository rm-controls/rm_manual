//
// Created by qiayuan on 7/25/21.
//

#include "rm_manual/engineer_manual.h"

namespace rm_manual
{
EngineerManual::EngineerManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalManual(nh, nh_referee)
  , operating_mode_(MANUAL)
  , action_client_("/engineer_middleware/move_steps", true)
{
  ROS_INFO("Waiting for middleware to start.");
  action_client_.waitForServer();
  ROS_INFO("Middleware started.");
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("power_on_calibration", rpc_value);
  power_on_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("mast_calibration", rpc_value);
  mast_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("arm_calibration", rpc_value);
  arm_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  left_switch_up_event_.setFalling(boost::bind(&EngineerManual::leftSwitchUpFall, this));
  left_switch_down_event_.setFalling(boost::bind(&EngineerManual::leftSwitchDownFall, this));
  ctrl_c_event_.setRising(boost::bind(&EngineerManual::ctrlCPress, this));
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));

  shift_c_event_.setRising(boost::bind(&EngineerManual::shiftCPress, this));

  c_event_.setRising(boost::bind(&EngineerManual::cPress, this));
  sentry_mode_ = 1;

  engineer_cmd_pub_ = nh.advertise<rm_msgs::EngineerCmd>("/engineer_cmd", 1);
}

void EngineerManual::run()
{
  ChassisGimbalManual::run();
  power_on_calibration_->update(ros::Time::now(), state_ != PASSIVE);
  mast_calibration_->update(ros::Time::now(), state_ != PASSIVE);
  arm_calibration_->update(ros::Time::now());
}

void EngineerManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::checkKeyboard(dbus_data);
  ctrl_c_event_.update(dbus_data->key_ctrl & dbus_data->key_c);
  ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);
  ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b);
  ctrl_f_event_.update(dbus_data->key_ctrl & dbus_data->key_f);
  ctrl_x_event_.update(dbus_data->key_ctrl & dbus_data->key_x);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
  ctrl_g_event_.update(dbus_data->key_ctrl & dbus_data->key_g);
  ctrl_s_event_.update(dbus_data->key_ctrl & dbus_data->key_s);
  ctrl_d_event_.update(dbus_data->key_ctrl & dbus_data->key_d);
  ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);
  ctrl_w_event_.update(dbus_data->key_ctrl & dbus_data->key_w);
  ctrl_e_event_.update(dbus_data->key_ctrl & dbus_data->key_e);

  shift_w_event_.update(dbus_data->key_shift & dbus_data->key_w);
  shift_s_event_.update(dbus_data->key_shift & dbus_data->key_s);
  shift_c_event_.update(dbus_data->key_shift & dbus_data->key_c);
  shift_x_event_.update(dbus_data->key_shift & dbus_data->key_x);

  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
}

void EngineerManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updateRc(dbus_data);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(dbus_data->s_l == rm_msgs::DbusData::DOWN);
}

void EngineerManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updatePc(dbus_data);
  vel_cmd_sender_->setAngularZVel(-dbus_data->m_x);
}

void EngineerManual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
  }
}

void EngineerManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}

void EngineerManual::chassisOutputOn()
{
  power_on_calibration_->reset();
  mast_calibration_->reset();
  if (MIDDLEWARE)
    action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchDownRise()
{
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::leftSwitchDownFall()
{
  mast_calibration_->reset();
  arm_calibration_->reset();
}

void EngineerManual::runStepQueue(const std::string& step_queue_name)
{
  rm_msgs::EngineerGoal goal;
  goal.step_queue_name = step_queue_name;
  engineer_cmd_data_.symbol = !engineer_cmd_data_.symbol;
  if (action_client_.isServerConnected())
  {
    if (operating_mode_ == MANUAL)
      action_client_.sendGoal(goal, boost::bind(&EngineerManual::actionDoneCallback, this, _1, _2),
                              boost::bind(&EngineerManual::actionActiveCallback, this),
                              boost::bind(&EngineerManual::actionFeedbackCallback, this, _1));
    operating_mode_ = MIDDLEWARE;
  }
  else
    ROS_ERROR("Can not connect to middleware");
}

void EngineerManual::actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback)
{
  engineer_cmd_data_.current_step_name = feedback->current_step;
  engineer_cmd_data_.finished_step = feedback->finished_step;
  engineer_cmd_data_.total_steps = feedback->total_steps;
  engineer_cmd_pub_.publish(engineer_cmd_data_);
}

void EngineerManual::actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                        const rm_msgs::EngineerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->finish);
  operating_mode_ = MANUAL;
}

void EngineerManual::cPress()
{
}

}  // namespace rm_manual
