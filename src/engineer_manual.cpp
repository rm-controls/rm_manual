//
// Created by qiayuan on 7/25/21.
//

#include "rm_manual/engineer_manual.h"

namespace rm_manual
{
EngineerManual::EngineerManual(ros::NodeHandle& nh)
  : ChassisGimbalManual(nh), operating_mode_(MANUAL), action_client_("/engineer_middleware/move_steps", true)
{
  ROS_INFO("Waiting for middleware to start.");
  action_client_.waitForServer();
  ROS_INFO("Middleware started.");
  // Command sender
  ros::NodeHandle nh_drag(nh, "drag");
  drag_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_drag);
  ros::NodeHandle nh_card(nh, "card");
  card_command_sender_ = new rm_common::CardCommandSender(nh_card);
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("power_on_calibration", rpc_value);
  power_on_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("arm_calibration", rpc_value);
  arm_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  left_switch_up_event_.setFalling(boost::bind(&EngineerManual::leftSwitchUpFall, this));
  left_switch_down_event_.setFalling(boost::bind(&EngineerManual::leftSwitchDownFall, this));
  ctrl_c_event_.setRising(boost::bind(&EngineerManual::ctrlCPress, this));
  ctrl_b_event_.setRising(boost::bind(&EngineerManual::ctrlBPress, this));
  ctrl_n_event_.setRising(boost::bind(&EngineerManual::ctrlNPress, this));
  ctrl_j_event_.setRising(boost::bind(&EngineerManual::ctrlJPress, this));
  ctrl_k_event_.setRising(boost::bind(&EngineerManual::ctrlKPress, this));
  ctrl_l_event_.setRising(boost::bind(&EngineerManual::ctrlLPress, this));
  ctrl_u_event_.setRising(boost::bind(&EngineerManual::ctrlUPress, this));
  ctrl_i_event_.setRising(boost::bind(&EngineerManual::ctrlIPress, this));
  ctrl_o_event_.setRising(boost::bind(&EngineerManual::ctrlOPress, this));
  ctrl_p_event_.setRising(boost::bind(&EngineerManual::ctrlPPress, this));
  ctrl_m_event_.setRising(boost::bind(&EngineerManual::ctrlMPress, this));
  ctrl_q_event_.setRising(boost::bind(&EngineerManual::ctrlQPress, this));
  ctrl_w_event_.setRising(boost::bind(&EngineerManual::ctrlWPress, this));
  ctrl_e_event_.setRising(boost::bind(&EngineerManual::ctrlEPress, this));
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));
  ctrl_t_event_.setRising(boost::bind(&EngineerManual::ctrlTPress, this));
  ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&EngineerManual::ctrlGPress, this));

  //  shift_w_event_.setRising(boost::bind(&EngineerManual::shiftWPress, this));
  //  shift_s_event_.setRising(boost::bind(&EngineerManual::shiftSPress, this));
  //  shift_c_event_.setRising(boost::bind(&EngineerManual::shiftCPress, this));
  //  shift_x_event_.setRising(boost::bind(&EngineerManual::shiftXPress, this));

  z_event_.setRising(boost::bind(&EngineerManual::zPress, this));
  x_event_.setRising(boost::bind(&EngineerManual::xPress, this));
  c_event_.setRising(boost::bind(&EngineerManual::cPress, this));
  sentry_mode_ = 1;
}

void EngineerManual::run()
{
  ChassisGimbalManual::run();
  power_on_calibration_->update(ros::Time::now(), state_ != PASSIVE);
  arm_calibration_->update(ros::Time::now());
}

void EngineerManual::checkKeyboard()
{
  ChassisGimbalManual::checkKeyboard();
  ctrl_c_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
  ctrl_b_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_b);
  ctrl_n_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_n);
  ctrl_j_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_j);
  ctrl_k_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_k);
  ctrl_l_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_l);
  ctrl_u_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_u);
  ctrl_i_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_i);
  ctrl_o_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_o);
  ctrl_p_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_p);
  ctrl_m_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_m);
  ctrl_q_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_q);
  ctrl_w_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_w);
  ctrl_e_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_e);
  ctrl_r_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  ctrl_t_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_t);
  ctrl_f_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_f);
  ctrl_g_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_g);

  //  shift_w_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_w);
  //  shift_s_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_s);
  //  shift_c_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_c);
  //  shift_x_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_x);

  z_event_.update(data_.dbus_data_.key_z & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  x_event_.update(data_.dbus_data_.key_x & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  c_event_.update(data_.dbus_data_.key_c & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
}

void EngineerManual::updateRc()
{
  ChassisGimbalManual::updateRc();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  left_switch_up_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN);
}

void EngineerManual::updatePc()
{
  ChassisGimbalManual::updatePc();
  vel_cmd_sender_->setAngularZVel(-data_.dbus_data_.m_x);
}

void EngineerManual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    gimbal_cmd_sender_->sendCommand(time);
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    drag_command_sender_->sendCommand(time);
    card_command_sender_->sendCommand(time);
  }
}

void EngineerManual::drawUi(const ros::Time& time)
{
  ChassisGimbalManual::drawUi(time);
  time_change_ui_->update("effort", time);
  time_change_ui_->update("temperature", time);
  trigger_change_ui_->update("drag", 0, drag_command_sender_->getState());
  if (data_.referee_.referee_data_.interactive_data.header_data_.data_cmd_id_ == 0x0201 &&
      data_.referee_.referee_data_.interactive_data.data_ != sentry_mode_)
    data_.referee_.sendInteractiveData(
        0x0200,
        data_.referee_.referee_data_.robot_color_ == "blue" ? rm_common::RobotId::BLUE_SENTRY : rm_common::RED_SENTRY,
        sentry_mode_);
  trigger_change_ui_->update("sentry", data_.referee_.referee_data_.interactive_data.data_, false);
  flash_ui_->update("calibration", time, power_on_calibration_->isCalibrated());
  if (!data_.joint_state_.name.empty())
    flash_ui_->update("card_warning", time, data_.joint_state_.effort[0] < 1.5);
  //    trigger_change_ui_->update("jog", jog_joint_name);
}

void EngineerManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}

void EngineerManual::chassisOutputOn()
{
  power_on_calibration_->reset();
  if (MIDDLEWARE)
    action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchDownRise()
{
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  if (card_command_sender_->getState())
    card_command_sender_->off();
  else
    card_command_sender_->long_on();
  //  action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  //  runStepQueue("BACK_HOME");
  if (card_command_sender_->getState())
    card_command_sender_->off();
  else
    card_command_sender_->short_on();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::leftSwitchDownFall()
{
  arm_calibration_->reset();
}

void EngineerManual::runStepQueue(const std::string& step_queue_name)
{
  rm_msgs::EngineerGoal goal;
  goal.step_queue_name = step_queue_name;
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
  trigger_change_ui_->update("queue", feedback->current_step);
  if (feedback->total_steps != 0)
    time_change_ui_->update("progress", ros::Time::now(), ((double)feedback->finished_step) / feedback->total_steps);
  else
    time_change_ui_->update("progress", ros::Time::now(), 0.);
}

void EngineerManual::actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                        const rm_msgs::EngineerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->finish);
  operating_mode_ = MANUAL;
}

void EngineerManual::zPress()
{
  if (card_command_sender_->getState())
    card_command_sender_->off();
  else
    card_command_sender_->long_on();
}

void EngineerManual::xPress()
{
  if (card_command_sender_->getState())
    card_command_sender_->off();
  else
    card_command_sender_->short_on();
}

void EngineerManual::cPress()
{
  if (drag_command_sender_->getState())
    drag_command_sender_->off();
  else
    drag_command_sender_->on();
}

}  // namespace rm_manual
