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
  // Servo
  ros::NodeHandle nh_servo(nh, "servo");
  servo_command_sender_ = new rm_common::Vel3DCommandSender(nh_servo);
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("power_on_calibration", rpc_value);
  power_on_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("arm_calibration", rpc_value);
  arm_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  left_switch_up_event_.setFalling(boost::bind(&EngineerManual::leftSwitchUpFall, this));
  left_switch_up_event_.setRising(boost::bind(&EngineerManual::leftSwitchUpRise, this));
  left_switch_down_event_.setFalling(boost::bind(&EngineerManual::leftSwitchDownFall, this));
  ctrl_q_event_.setRising(boost::bind(&EngineerManual::ctrlQPress, this));
  ctrl_a_event_.setRising(boost::bind(&EngineerManual::ctrlAPress, this));
  ctrl_z_event_.setRising(boost::bind(&EngineerManual::ctrlZPress, this));
  ctrl_w_event_.setRising(boost::bind(&EngineerManual::ctrlWPress, this));
  ctrl_s_event_.setRising(boost::bind(&EngineerManual::ctrlSPress, this));
  ctrl_x_event_.setRising(boost::bind(&EngineerManual::ctrlXPress, this));
  ctrl_e_event_.setRising(boost::bind(&EngineerManual::ctrlEPress, this));
  ctrl_d_event_.setRising(boost::bind(&EngineerManual::ctrlDPress, this));
  ctrl_c_event_.setRising(boost::bind(&EngineerManual::ctrlCPress, this));
  ctrl_b_event_.setRising(boost::bind(&EngineerManual::ctrlBPress, this));
  ctrl_b_event_.setFalling(boost::bind(&EngineerManual::ctrlBRelease, this));
  ctrl_v_event_.setRising(boost::bind(&EngineerManual::ctrlVPress, this));
  z_event_.setRising(boost::bind(&EngineerManual::ZPress, this));
  x_event_.setRising(boost::bind(&EngineerManual::XPress, this));
  c_event_.setRising(boost::bind(&EngineerManual::CPress, this));
  v_event_.setRising(boost::bind(&EngineerManual::VPress, this));
  b_event_.setRising(boost::bind(&EngineerManual::BPress, this));
  shift_z_event_.setRising(boost::bind(&EngineerManual::shiftZPress, this));
  shift_x_event_.setRising(boost::bind(&EngineerManual::shiftXPress, this));
  shift_c_event_.setRising(boost::bind(&EngineerManual::shiftCPress, this));
  shift_v_event_.setRising(boost::bind(&EngineerManual::shiftVPress, this));
  shift_b_event_.setRising(boost::bind(&EngineerManual::shiftBPress, this));
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));
  shift_q_event_.setRising(boost::bind(&EngineerManual::shiftQPress, this));
  //  shift_w_event_.setRising(boost::bind(&EngineerManual::shiftWPress, this));
  shift_e_event_.setRising(boost::bind(&EngineerManual::shiftEPress, this));
  shift_r_event_.setRising(boost::bind(&EngineerManual::shiftRPress, this));
  ctrl_g_event_.setRising(boost::bind(&EngineerManual::ctrlGPress, this));
  ctrl_g_event_.setFalling(boost::bind(&EngineerManual::ctrlGRelease, this));
  ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
  shift_event_.setActiveHigh(boost::bind(&EngineerManual::shiftPressing, this));
  shift_event_.setFalling(boost::bind(&EngineerManual::shiftRelease, this));
}

void EngineerManual::run()
{
  ChassisGimbalManual::run();
  power_on_calibration_->update(ros::Time::now(), state_ != PASSIVE);
  arm_calibration_->update(ros::Time::now());
  updateServo();
}

void EngineerManual::checkKeyboard()
{
  ChassisGimbalManual::checkKeyboard();
  ctrl_q_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_q);
  ctrl_a_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_a);
  ctrl_z_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_z);
  ctrl_w_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_w);
  ctrl_s_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_s);
  ctrl_x_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_x);
  ctrl_e_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_e);
  ctrl_d_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_d);
  ctrl_c_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_c);
  ctrl_b_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_b);
  ctrl_v_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_v);
  z_event_.update(data_.dbus_data_.key_z & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  x_event_.update(data_.dbus_data_.key_x & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  c_event_.update(data_.dbus_data_.key_c & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  v_event_.update(data_.dbus_data_.key_v & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  b_event_.update(data_.dbus_data_.key_b & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  z_event_.update(data_.dbus_data_.key_z & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  shift_z_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_z);
  shift_x_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_x);
  shift_c_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_c);
  shift_v_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_v);
  shift_b_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_b);
  ctrl_r_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  shift_q_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_q);
  //  shift_w_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_w);
  shift_e_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_e);
  shift_r_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_r);
  ctrl_g_event_.update(data_.dbus_data_.key_g & data_.dbus_data_.key_ctrl);
  ctrl_f_event_.update(data_.dbus_data_.key_f & data_.dbus_data_.key_ctrl);
  shift_event_.update(data_.dbus_data_.key_shift & !data_.dbus_data_.key_ctrl);
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
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  //  vel_cmd_sender_->setAngularZVel(-data_.dbus_data_.m_x);
}

void EngineerManual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    drag_command_sender_->sendCommand(time);
    card_command_sender_->sendCommand(time);
  }
  if (servo_mode_ == SERVO)
    servo_command_sender_->sendCommand(time);
}

void EngineerManual::drawUi(const ros::Time& time)
{
  ChassisGimbalManual::drawUi(time);
  time_change_ui_->update("effort", time);
  time_change_ui_->update("temperature", time);
  trigger_change_ui_->update("drag", 0, drag_command_sender_->getState());
  trigger_change_ui_->update("long_card", 0, card_command_sender_->getState());
  trigger_change_ui_->update("short_card", 0, card_command_sender_->getState());
  flash_ui_->update("calibration", time, power_on_calibration_->isCalibrated());
  if (!data_.joint_state_.name.empty())
    flash_ui_->update("card_warning", time, data_.joint_state_.effort[0] < 1.5);
  //    trigger_change_ui_->update("jog", jog_joint_name);
}

void EngineerManual::updateServo()
{
  servo_command_sender_->setLinearVel(data_.dbus_data_.ch_l_x, data_.dbus_data_.ch_l_y, data_.dbus_data_.wheel);
  servo_command_sender_->setAngularVel(angular_z_scale_, data_.dbus_data_.ch_r_x, data_.dbus_data_.ch_r_y);
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
  servo_mode_ = SERVO;
  action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  servo_mode_ = JOINT;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::rightSwitchUpRise()
{
  servo_mode_ = SERVO;
  ChassisGimbalManual::rightSwitchUpRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::leftSwitchUpRise()
{
}

void EngineerManual::leftSwitchUpFall()
{
  runStepQueue("BACK_HOME");
  trigger_change_ui_->update("step", "BACK_HOME");
}

void EngineerManual::leftSwitchDownFall()
{
  arm_calibration_->reset();
  //  power_on_calibration_->reset();
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

void EngineerManual::ctrlFPress()
{
  runStepQueue(toward_ + situation_);
}

void EngineerManual::ctrlGPress()
{
  angular_z_scale_ = 0.5;
  //  trigger_change_ui_->update("finished" + situation_);
}
void EngineerManual::ctrlGRelease()
{
  angular_z_scale_ = 0;
  //  trigger_change_ui_->update("finished" + situation_);
}

void EngineerManual::ctrlQPress()
{
  toward_ = "LF_";
  //  trigger_change_ui_->update("step", toward_ + process_);
}

void EngineerManual::ctrlAPress()
{
  toward_ = "MID_";
  //  trigger_change_ui_->update("step", prefix_ + toward_ + process_);
}

void EngineerManual::ctrlZPress()
{
  toward_ = "RT_";
  //  trigger_change_ui_->update("step", prefix_ + toward_ + process_);
}

void EngineerManual::ctrlWPress()
{
  //  trigger_change_ui_->update("step", prefix_ + toward_ + process_);
}

void EngineerManual::ctrlSPress()
{
  runStepQueue("CHASSIS3");
  //  trigger_change_ui_->update("step", prefix_ + toward_ + process_);
}

void EngineerManual::ctrlXPress()
{
  situation_ = "SKY_ISLAND";
  //  trigger_change_ui_->update("step", prefix_ + toward_ + process_);
}

void EngineerManual::ctrlEPress()
{
  runStepQueue("CHASSIS");
  //  if (process_ == "PROC")
  //    process_ = "AFTER";
  //  else
  //    process_ = "PRE";
  //  runStepQueue(prefix_ + toward_ + process_);
  //  trigger_change_ui_->update("step", prefix_ + toward_ + process_);
}

void EngineerManual::ctrlDPress()
{
  toward_ = "";
  situation_ = "STORE";
  //  process_ = "PROC";
  //  runStepQueue(prefix_ + toward_ + process_);
  //  trigger_change_ui_->update("step", prefix_ + toward_ + process_);
}

void EngineerManual::ctrlBPress()
{
  angular_z_scale_ = -0.5;
  //  runStepQueue("BACK_HOME");
  //  trigger_change_ui_->update("step", "BACK_HOME");
  //  process_ = "0";
}

void EngineerManual::ctrlBRelease()
{
  angular_z_scale_ = 0.0;
  //  runStepQueue("BACK_HOME");
  //  trigger_change_ui_->update("step", "BACK_HOME");
  //  process_ = "0";
}

void EngineerManual::ctrlVPress()
{
  toward_ = "";
  situation_ = "GET_STORE";
  //  std::cout << process_ << std::endl;
}

void EngineerManual::ZPress()
{
  if (card_command_sender_->getState())
  {
    card_command_sender_->off();
    trigger_change_ui_->update("long_card", "off");
  }
  else
  {
    card_command_sender_->short_on();
    trigger_change_ui_->update("long_card", "on");
  }
  //  runStepQueue("STORE");
  //  trigger_change_ui_->update("step", "STORE");
}

void EngineerManual::XPress()
{
  if (card_command_sender_->getState())
  {
    card_command_sender_->off();
    trigger_change_ui_->update("long_card", "off");
  }
  else
  {
    card_command_sender_->short_on();
    trigger_change_ui_->update("long_card", "on");
  }
  //  runStepQueue("GET_STONE");
  //  trigger_change_ui_->update("step", "GET_STONE");
}

void EngineerManual::CPress()
{
  if (drag_command_sender_->getState())
  {
    drag_command_sender_->off();
    trigger_change_ui_->update("drag", "off");
  }
  else
  {
    drag_command_sender_->on();
    trigger_change_ui_->update("drag", "on");
  }
  //  runStepQueue("GET_STONE_SKY");
  //  trigger_change_ui_->update("step", "GET_STONE_SKY");
}

void EngineerManual::VPress()
{
  runStepQueue("OPEN_GRIPPER");
  trigger_change_ui_->update("step", "OPEN_GRIPPER");
  //  runStepQueue("SMALL_STONE");
  //  trigger_change_ui_->update("step", "SMALL_STONE");
}

void EngineerManual::BPress()
{
  runStepQueue("CLOSE_GRIPPER");
  trigger_change_ui_->update("step", "CLOSE_GRIPPER");
  //  runStepQueue("GROUND_STONE");
  //  trigger_change_ui_->update("step", "GROUND_STONE");
}

void EngineerManual::shiftZPress()
{
  //    if (drag_command_sender_->getState())
  //    {
  //      drag_command_sender_->off();
  //      trigger_change_ui_->update("drag", "off");
  //    }
  //    else
  //    {
  //      drag_command_sender_->on();
  //      trigger_change_ui_->update("drag", "on");
  //    }
}

void EngineerManual::shiftXPress()
{
  //    if (card_command_sender_->getState())
  //    {
  //      card_command_sender_->off();
  //      trigger_change_ui_->update("long_card", "off");
  //    }
  //    else
  //    {
  //      card_command_sender_->short_on();
  //      trigger_change_ui_->update("long_card", "on");
  //    }
}

void EngineerManual::shiftCPress()
{
  //    if (card_command_sender_->getState())
  //    {
  //      card_command_sender_->off();
  //      trigger_change_ui_->update("short_card", "off");
  //    }
  //    else
  //    {
  //      card_command_sender_->long_on();
  //      trigger_change_ui_->update("short_card", "on");
  //    }
}

void EngineerManual::shiftVPress()
{
  runStepQueue("OPEN_GRIPPER");
  trigger_change_ui_->update("step", "OPEN_GRIPPER");
}

void EngineerManual::shiftBPress()
{
  runStepQueue("CLOSE_GRIPPER");
  trigger_change_ui_->update("step", "CLOSE_GRIPPER");
}

void EngineerManual::shiftRPress()
{
  runStepQueue("SKY_GIMBAL");
  trigger_change_ui_->update("step", "SKY_GIMBAL");
}

void EngineerManual::shiftQPress()
{
  runStepQueue("WALKING_GIMBAL");
  trigger_change_ui_->update("step", "WALKING_GIMBAL");
}

//

void EngineerManual::shiftEPress()
{
  runStepQueue("BACK_POS");
  trigger_change_ui_->update("step", "BACK_POS");
}

void EngineerManual::ctrlCPress()
{
  action_client_.cancelAllGoals();
  runStepQueue("DELETE_SCENE");
  trigger_change_ui_->update("step", "DELETE_SCENE and CANCEL");
}

void EngineerManual::ctrlRPress()
{
  arm_calibration_->reset();
  power_on_calibration_->reset();
}
void EngineerManual::shiftPressing()
{
  speed_change_mode_ = 1;
}

void EngineerManual::shiftRelease()
{
  speed_change_mode_ = 0;
}

}  // namespace rm_manual
