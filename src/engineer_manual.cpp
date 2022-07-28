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
  servo_reset_caller_ = new rm_common::ServiceCallerBase<std_srvs::Empty>(nh_servo, "/servo_server/reset_servo_status");
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
  ctrl_v_event_.setRising(boost::bind(&EngineerManual::ctrlVPress, this));
  ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&EngineerManual::ctrlGPress, this));
  z_event_.setRising(boost::bind(&EngineerManual::zPress, this));
  x_event_.setRising(boost::bind(&EngineerManual::xPress, this));
  c_event_.setRising(boost::bind(&EngineerManual::cPress, this));
  v_event_.setRising(boost::bind(&EngineerManual::vPress, this));
  b_event_.setRising(boost::bind(&EngineerManual::bPress, this));
  r_event_.setRising(boost::bind(&EngineerManual::rPress, this));
  g_event_.setRising(boost::bind(&EngineerManual::gPress, this));
  g_event_.setFalling(boost::bind(&EngineerManual::gRelease, this));
  f_event_.setRising(boost::bind(&EngineerManual::fPress, this));
  f_event_.setFalling(boost::bind(&EngineerManual::fRelease, this));
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));
  shift_q_event_.setRising(boost::bind(&EngineerManual::shiftQPress, this));
  shift_q_event_.setFalling(boost::bind(&EngineerManual::shiftQRelease, this));
  shift_e_event_.setRising(boost::bind(&EngineerManual::shiftEPress, this));
  shift_e_event_.setFalling(boost::bind(&EngineerManual::shiftERelease, this));
  shift_z_event_.setRising(boost::bind(&EngineerManual::shiftZPress, this));
  shift_x_event_.setRising(boost::bind(&EngineerManual::shiftXPress, this));
  shift_c_event_.setRising(boost::bind(&EngineerManual::shiftCPress, this));
  shift_v_event_.setRising(boost::bind(&EngineerManual::shiftVPress, this));
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
  ctrl_r_event_.update(data_.dbus_data_.key_ctrl & data_.dbus_data_.key_r);
  ctrl_g_event_.update(data_.dbus_data_.key_g & data_.dbus_data_.key_ctrl);
  ctrl_f_event_.update(data_.dbus_data_.key_f & data_.dbus_data_.key_ctrl);

  z_event_.update(data_.dbus_data_.key_z & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  x_event_.update(data_.dbus_data_.key_x & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  c_event_.update(data_.dbus_data_.key_c & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  v_event_.update(data_.dbus_data_.key_v & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  b_event_.update(data_.dbus_data_.key_b & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  g_event_.update(data_.dbus_data_.key_g & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);
  f_event_.update(data_.dbus_data_.key_f & !data_.dbus_data_.key_ctrl & !data_.dbus_data_.key_shift);

  shift_z_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_z);
  shift_x_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_x);
  shift_c_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_c);
  shift_v_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_v);
  shift_b_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_b);
  shift_q_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_q);
  shift_e_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_e);
  shift_r_event_.update(data_.dbus_data_.key_shift & data_.dbus_data_.key_r);
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
  vel_cmd_sender_->setAngularZVel(-data_.dbus_data_.m_x);
}

void EngineerManual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    drag_command_sender_->sendCommand(time);
    card_command_sender_->sendCommand(time);
  }
  if (servo_mode_ == SERVO)
    servo_command_sender_->sendCommand(time);
  if (gimbal_mode_ == RATE)
  {
    gimbal_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->setZero();
    vel_cmd_sender_->sendCommand(time);
  }
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
  servo_command_sender_->setLinearVel(data_.dbus_data_.ch_l_y, -data_.dbus_data_.ch_l_x, -data_.dbus_data_.wheel);
  servo_command_sender_->setAngularVel(-data_.dbus_data_.ch_r_x, -data_.dbus_data_.ch_r_y, angular_z_scale_);
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
  servo_reset_caller_->callService();
  action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  servo_mode_ = JOINT;
  gimbal_mode_ = DIRECT;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  gimbal_mode_ = DIRECT;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::leftSwitchUpRise()
{
}

void EngineerManual::leftSwitchUpFall()
{
  runStepQueue("EMPTY_HOME");
  trigger_change_ui_->update("step", "NORMAL_HOME");
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
  trigger_change_ui_->update("step", "Done " + prefix_ + root_ + " press next");
  std::cout << "Done! " + prefix_ + root_ << std::endl;
  operating_mode_ = MANUAL;
}

void EngineerManual::judgePrefix()
{
  switch (root_num_)
  {
    case (0):
      if (prefix_num_ == 1)
        prefix_ = "WAIT1_";
      if (prefix_num_ == 2)
        prefix_ = "WAIT2_";
      if (prefix_num_ == 3)
        prefix_ = "WAIT3_";
      if (prefix_num_ == 4)
        prefix_ = "WAIT4_";
      break;
    case (1):
      if (prefix_num_ == 1)
        prefix_ = "LF_";
      if (prefix_num_ == 2)
        prefix_ = "MID_";
      if (prefix_num_ == 3)
        prefix_ = "RT_";
      if (prefix_num_ == 4)
        prefix_ = "READY_";
      break;
    case (2):
      if (prefix_num_ == 1)
        prefix_ = "SKY_";
      if (prefix_num_ == 2)
        prefix_ = "NORMAL_";
      if (prefix_num_ == 3)
        prefix_ = "NO!!";
      if (prefix_num_ == 4)
        prefix_ = "NO!!";
      break;
    case (3):
      if (prefix_num_ == 1)
        prefix_ = "EMPTY_";
      if (prefix_num_ == 2)
        prefix_ = "STORED_";
      if (prefix_num_ == 3)
        prefix_ = "NO!!";
      if (prefix_num_ == 4)
        prefix_ = "NO!!";
      break;
  }
}
void EngineerManual::judgeRoot()
{
  switch (prefix_num_)
  {
    case (0):
      if (root_num_ == 1)
        prefix_ = "WAIT_";
      if (root_num_ == 2)
        prefix_ = "WAIT_";
      if (root_num_ == 3)
        prefix_ = "WAIT_";
      break;
    case (1):
      if (root_num_ == 1)
        prefix_ = "LF_";
      if (root_num_ == 2)
        prefix_ = "SKY_";
      if (root_num_ == 3)
        prefix_ = "EMPTY_";
      break;
    case (2):
      if (root_num_ == 1)
        prefix_ = "MID_";
      if (root_num_ == 2)
        prefix_ = "NORMAL_";
      if (root_num_ == 3)
        prefix_ = "STORED_";
      break;
    case (3):
      if (root_num_ == 1)
        prefix_ = "RT_";
      if (root_num_ == 2)
        prefix_ = "NO!!";
      if (root_num_ == 3)
        prefix_ = "NO!!";
    case (4):
      if (root_num_ == 1)
        prefix_ = "READY_";
      if (root_num_ == 2)
        prefix_ = "NO!!";
      if (root_num_ == 3)
        prefix_ = "NO!!";
      break;
  }
}

void EngineerManual::ctrlGPress()
{
  runStepQueue(prefix_ + root_);
  trigger_change_ui_->update("step", "Finished " + prefix_ + root_);
  std::cout << "do " + prefix_ + root_ << std::endl;
}
void EngineerManual::ctrlFPress()
{
  root_ += "0";
  runStepQueue(prefix_ + root_);
  trigger_change_ui_->update("step", "Finished " + prefix_ + root_);
  std::cout << "do " + prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlQPress()
{
  prefix_num_ = 1;
  judgePrefix();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlWPress()
{
  prefix_num_ = 2;
  judgePrefix();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlEPress()
{
  prefix_num_ = 3;
  judgePrefix();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlRPress()
{
  prefix_num_ = 4;
  judgePrefix();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlAPress()
{
  root_num_ = 1;
  root_ = "SKY_ISLAND";
  judgeRoot();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlSPress()
{
  root_num_ = 1;
  root_ = "BIG_ISLAND";
  judgeRoot();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::bPress()
{
  root_num_ = 1;
  root_ = "GROUND_STONE";
  judgeRoot();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlXPress()
{
  root_num_ = 2;
  root_ = "GAIN_STORE_STONE";
  judgeRoot();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}
void EngineerManual::ctrlDPress()
{
  prefix_ = "";
  root_ = "EXCHANGE";
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlZPress()
{
  prefix_ = "";
  root_ = "STORE_STONE";
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::ctrlVPress()
{
  prefix_ = "";
  root_ = "SMALL_ISLAND";
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::zPress()
{
  if (card_command_sender_->getState())
  {
    card_command_sender_->off();
    trigger_change_ui_->update("long_card", "off");
    std::cout << "long_card off" << std::endl;
  }
  else
  {
    card_command_sender_->short_on();
    trigger_change_ui_->update("long_card", "on");
    std::cout << "long_card on" << std::endl;
  }
}

void EngineerManual::xPress()
{
  if (card_command_sender_->getState())
  {
    card_command_sender_->off();
    trigger_change_ui_->update("short_card", "off");
    std::cout << "short_card off" << std::endl;
  }
  else
  {
    card_command_sender_->short_on();
    trigger_change_ui_->update("short_card", "on");
    std::cout << "short_card on" << std::endl;
  }
}

void EngineerManual::cPress()
{
  if (drag_command_sender_->getState())
  {
    drag_command_sender_->off();
    trigger_change_ui_->update("drag", "off");
    std::cout << "drag off" << std::endl;
  }
  else
  {
    drag_command_sender_->on();
    trigger_change_ui_->update("drag", "on");
    std::cout << "drag on" << std::endl;
  }
}

void EngineerManual::ctrlCPress()
{
  action_client_.cancelAllGoals();
  runStepQueue("DELETE_SCENE");
  trigger_change_ui_->update("step", "DELETE_SCENE and CANCEL");
  std::cout << "DELETE_SCENE and CANCEL" << std::endl;
}

void EngineerManual::rPress()
{
  arm_calibration_->reset();
  power_on_calibration_->reset();
  trigger_change_ui_->update("step", "calibratied");
  std::cout << "Calibrated" << std::endl;
}

void EngineerManual::ctrlBPress()
{
  root_num_ = 3;
  root_ = "BACK_HOME";
  judgePrefix();
  trigger_change_ui_->update("step", prefix_ + root_);
  std::cout << prefix_ + root_ << std::endl;
}

void EngineerManual::vPress()
{
  {
    servo_mode_ = SERVO;
    servo_reset_caller_->callService();
    trigger_change_ui_->update("step", "ENTER SERVO");
    std::cout << "ENTER SERVO" << std::endl;
  }
  trigger_change_ui_->update("step", "servo mode controlling");
}

void EngineerManual::shiftPressing()
{
  speed_change_mode_ = 1;
  trigger_change_ui_->update("step", "low speed mode");
  std::cout << "low speed mode" << std::endl;
}
void EngineerManual::shiftRelease()
{
  speed_change_mode_ = 0;
  trigger_change_ui_->update("step", "low speed cancle");
  std::cout << "low speed cancle" << std::endl;
}

void EngineerManual::shiftQPress()
{
  angular_z_scale_ = 0.5;
}
void EngineerManual::shiftQRelease()
{
  angular_z_scale_ = 0.;
}

void EngineerManual::shiftEPress()
{
  angular_z_scale_ = -0.5;
}
void EngineerManual::shiftERelease()
{
  angular_z_scale_ = 0.;
}

void EngineerManual::gPress()
{
  runStepQueue("CLOSE_GRIPPER");
  trigger_change_ui_->update("step", "close gripper");
  std::cout << "close gripper" << std::endl;
}
void EngineerManual::gRelease()
{
  runStepQueue("OPEN_GRIPPER");
  trigger_change_ui_->update("step", "open gripper");
  std::cout << "open gripper" << std::endl;
}
void EngineerManual::fPress()
{
  // enter gimbal rate
  gimbal_mode_ = RATE;
  trigger_change_ui_->update("step", "gimbal rate");
  std::cout << "enter rate" << std::endl;
}
void EngineerManual::fRelease()
{
  // exit gimbal rate
  gimbal_mode_ = DIRECT;
  trigger_change_ui_->update("step", "gimbal direct");
  std::cout << "exie rate" << std::endl;
}
void EngineerManual::shiftZPress()
{
  runStepQueue("WALKING_POS");
  trigger_change_ui_->update("step", "WALKING_POS");
  std::cout << "WALKING_POS" << std::endl;
}
void EngineerManual::shiftXPress()
{
  runStepQueue("BIG_STONE_POS");
  trigger_change_ui_->update("step", "BIG_STONE_POS");
  std::cout << "BIG_STONE_POS" << std::endl;
}
void EngineerManual::shiftCPress()
{
  runStepQueue("BACK_POS");
  trigger_change_ui_->update("step", "BACK_POS");
  std::cout << "BACK_POS" << std::endl;
}
void EngineerManual::shiftVPress()
{
  runStepQueue("SKY_POS");
  trigger_change_ui_->update("step", "SKY_POS");
  std::cout << "SKY_POS" << std::endl;
}
}  // namespace rm_manual
