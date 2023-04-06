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
  exchange_sub_ = nh.subscribe<rm_msgs::ExchangerMsg>("/pnp_publisher", 10, &EngineerManual::exchangeCallback, this);
  ROS_INFO("Waiting for middleware to start.");
  action_client_.waitForServer();
  ROS_INFO("Middleware started.");
  // UI
  ui_send_ = nh.advertise<rm_msgs::EngineerUi>("/engineer_ui", 10);
  // Drag
  ros::NodeHandle nh_drag(nh, "drag");
  drag_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_drag);
  // Reversal
  ros::NodeHandle nh_reversal(nh, "reversal");
  reversal_command_sender_ = new rm_common::MultiDofCommandSender(nh_reversal);
  // Servo
  ros::NodeHandle nh_servo(nh, "servo");
  servo_command_sender_ = new rm_common::Vel3DCommandSender(nh_servo);
  servo_reset_caller_ = new rm_common::ServiceCallerBase<std_srvs::Empty>(nh_servo, "/servo_server/reset_servo_status");
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("calibration_gather", rpc_value);
  calibration_gather = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  left_switch_up_event_.setFalling(boost::bind(&EngineerManual::leftSwitchUpFall, this));
  left_switch_up_event_.setRising(boost::bind(&EngineerManual::leftSwitchUpRise, this));
  left_switch_down_event_.setFalling(boost::bind(&EngineerManual::leftSwitchDownFall, this));
  ctrl_q_event_.setRising(boost::bind(&EngineerManual::ctrlQPress, this));
  ctrl_a_event_.setRising(boost::bind(&EngineerManual::ctrlAPress, this));
  ctrl_z_event_.setRising(boost::bind(&EngineerManual::ctrlZPress, this));
  ctrl_z_event_.setActiveHigh(boost::bind(&EngineerManual::ctrlZPressing, this));
  ctrl_z_event_.setFalling(boost::bind(&EngineerManual::ctrlZRelease, this));
  ctrl_w_event_.setRising(boost::bind(&EngineerManual::ctrlWPress, this));
  ctrl_s_event_.setRising(boost::bind(&EngineerManual::ctrlSPress, this));
  ctrl_x_event_.setRising(boost::bind(&EngineerManual::ctrlXPress, this));
  ctrl_e_event_.setRising(boost::bind(&EngineerManual::ctrlEPress, this));
  ctrl_d_event_.setRising(boost::bind(&EngineerManual::ctrlDPress, this));
  ctrl_c_event_.setRising(boost::bind(&EngineerManual::ctrlCPress, this));
  ctrl_v_event_.setRising(boost::bind(&EngineerManual::ctrlVPress, this));
  ctrl_v_event_.setFalling(boost::bind(&EngineerManual::ctrlVRelease, this));
  ctrl_b_event_.setRising(boost::bind(&EngineerManual::ctrlBPress, this));
  ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&EngineerManual::ctrlGPress, this));
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));
  e_event_.setActiveHigh(boost::bind(&EngineerManual::ePressing, this));
  q_event_.setActiveHigh(boost::bind(&EngineerManual::qPressing, this));
  e_event_.setFalling(boost::bind(&EngineerManual::eRelease, this));
  q_event_.setFalling(boost::bind(&EngineerManual::qRelease, this));
  z_event_.setActiveHigh(boost::bind(&EngineerManual::zPressing, this));
  x_event_.setRising(boost::bind(&EngineerManual::xPress, this));
  z_event_.setFalling(boost::bind(&EngineerManual::zRelease, this));
  c_event_.setActiveHigh(boost::bind(&EngineerManual::cPressing, this));
  c_event_.setFalling(boost::bind(&EngineerManual::cRelease, this));
  r_event_.setRising(boost::bind(&EngineerManual::rPress, this));
  v_event_.setActiveHigh(boost::bind(&EngineerManual::vPressing, this));
  v_event_.setFalling(boost::bind(&EngineerManual::vRelease, this));
  g_event_.setActiveHigh(boost::bind(&EngineerManual::gPressing, this));
  g_event_.setFalling(boost::bind(&EngineerManual::gRelease, this));
  b_event_.setActiveHigh(boost::bind(&EngineerManual::bPressing, this));
  b_event_.setFalling(boost::bind(&EngineerManual::bRelease, this));
  f_event_.setActiveHigh(boost::bind(&EngineerManual::fPressing, this));
  f_event_.setFalling(boost::bind(&EngineerManual::fRelease, this));
  shift_z_event_.setRising(boost::bind(&EngineerManual::shiftZPress, this));
  shift_c_event_.setRising(boost::bind(&EngineerManual::shiftCPress, this));
  shift_v_event_.setRising(boost::bind(&EngineerManual::shiftVPress, this));
  shift_v_event_.setFalling(boost::bind(&EngineerManual::shiftVRelease, this));
  shift_b_event_.setRising(boost::bind(&EngineerManual::shiftBPress, this));
  shift_b_event_.setFalling(boost::bind(&EngineerManual::shiftBRelease, this));
  shift_x_event_.setRising(boost::bind(&EngineerManual::shiftXPress, this));
  shift_g_event_.setRising(boost::bind(&EngineerManual::shiftGPress, this));
  shift_f_event_.setRising(boost::bind(&EngineerManual::shiftFPress, this));
  shift_r_event_.setRising(boost::bind(&EngineerManual::shiftRPress, this));
  shift_event_.setActiveHigh(boost::bind(&EngineerManual::shiftPressing, this));
  shift_event_.setFalling(boost::bind(&EngineerManual::shiftRelease, this));
  mouse_left_event_.setFalling(boost::bind(&EngineerManual::mouseLeftRelease, this));
  mouse_right_event_.setFalling(boost::bind(&EngineerManual::mouseRightRelease, this));
}

void EngineerManual::run()
{
  ChassisGimbalManual::run();
  calibration_gather->update(ros::Time::now(), state_ != PASSIVE);
  sendUi(prefix_ + root_, reversal_state_, drag_state_, stone_num_, joint_temperature_, gripper_state_);
}

void EngineerManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::checkKeyboard(dbus_data);
  ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);
  ctrl_a_event_.update(dbus_data->key_ctrl & dbus_data->key_a);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);
  ctrl_w_event_.update(dbus_data->key_ctrl & dbus_data->key_w);
  ctrl_s_event_.update(dbus_data->key_ctrl & dbus_data->key_s);
  ctrl_x_event_.update(dbus_data->key_ctrl & dbus_data->key_x);
  ctrl_e_event_.update(dbus_data->key_ctrl & dbus_data->key_e);
  ctrl_d_event_.update(dbus_data->key_ctrl & dbus_data->key_d);
  ctrl_c_event_.update(dbus_data->key_ctrl & dbus_data->key_c);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
  ctrl_e_event_.update(dbus_data->key_ctrl & dbus_data->key_e);
  ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b);
  ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
  ctrl_g_event_.update(dbus_data->key_g & dbus_data->key_ctrl);
  ctrl_f_event_.update(dbus_data->key_f & dbus_data->key_ctrl);

  z_event_.update(dbus_data->key_z & !dbus_data->key_ctrl & !dbus_data->key_shift);
  x_event_.update(dbus_data->key_x & !dbus_data->key_ctrl & !dbus_data->key_shift);
  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
  v_event_.update(dbus_data->key_v & !dbus_data->key_ctrl & !dbus_data->key_shift);
  b_event_.update(dbus_data->key_b & !dbus_data->key_ctrl & !dbus_data->key_shift);
  g_event_.update(dbus_data->key_g & !dbus_data->key_ctrl & !dbus_data->key_shift);
  f_event_.update(dbus_data->key_f & !dbus_data->key_ctrl & !dbus_data->key_shift);
  r_event_.update(dbus_data->key_r & !dbus_data->key_ctrl & !dbus_data->key_shift);
  q_event_.update(dbus_data->key_q & !dbus_data->key_ctrl);
  e_event_.update(dbus_data->key_e & !dbus_data->key_ctrl);

  shift_z_event_.update(dbus_data->key_shift & dbus_data->key_z);
  shift_x_event_.update(dbus_data->key_shift & dbus_data->key_x & !dbus_data->key_ctrl);
  shift_c_event_.update(dbus_data->key_shift & dbus_data->key_c);
  shift_v_event_.update(dbus_data->key_shift & dbus_data->key_v);
  shift_b_event_.update(dbus_data->key_shift & dbus_data->key_b);
  shift_q_event_.update(dbus_data->key_shift & dbus_data->key_q);
  shift_e_event_.update(dbus_data->key_shift & dbus_data->key_e);
  shift_r_event_.update(dbus_data->key_shift & dbus_data->key_r);
  shift_g_event_.update(dbus_data->key_shift & dbus_data->key_g);
  shift_x_event_.update(dbus_data->key_shift & dbus_data->key_x);
  shift_f_event_.update(dbus_data->key_shift & dbus_data->key_f);
  shift_event_.update(dbus_data->key_shift & !dbus_data->key_ctrl);

  mouse_left_event_.update(dbus_data->p_l);
  mouse_right_event_.update(dbus_data->p_r);

  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
}

void EngineerManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
  updateServo(data);
}

void EngineerManual::exchangeCallback(const rm_msgs::ExchangerMsg ::ConstPtr& data)
{
  is_exchange_ = data->flag;
  target_shape_ = data->shape;
}

void EngineerManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updateRc(dbus_data);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(dbus_data->s_l == rm_msgs::DbusData::DOWN);
}

void EngineerManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updatePc(dbus_data);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  if (!reversal_motion_)
    reversal_command_sender_->setGroupVel(0., 0., 5 * dbus_data->ch_r_y, 5 * dbus_data->ch_l_x, 5 * dbus_data->ch_l_y,
                                          0.);
}

void EngineerManual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    reversal_command_sender_->sendCommand(time);
    drag_command_sender_->sendCommand(time);
  }
  if (servo_mode_ == SERVO)
  {
    servo_command_sender_->sendCommand(time);
    reversal_command_sender_->setZero();
  }
  if (gimbal_mode_ == RATE)
    gimbal_cmd_sender_->sendCommand(time);
}

void EngineerManual::updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  servo_command_sender_->setLinearVel(dbus_data->ch_l_y, -dbus_data->ch_l_x, -dbus_data->wheel);
  servo_command_sender_->setAngularVel(dbus_data->ch_r_x, dbus_data->ch_r_y, angular_z_scale_);
  z_event_.update(dbus_data->key_z & !dbus_data->key_ctrl & !dbus_data->key_shift);
  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
}

void EngineerManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}

void EngineerManual::chassisOutputOn()
{
  if (MIDDLEWARE)
    action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchDownRise()
{
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
  servo_mode_ = SERVO;
  gimbal_mode_ = DIRECT;
  servo_reset_caller_->callService();
  action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  servo_mode_ = JOINT;
  gimbal_mode_ = RATE;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
}

void EngineerManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  gimbal_mode_ = DIRECT;
  servo_mode_ = JOINT;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::GYRO);
}

void EngineerManual::leftSwitchUpRise()
{
  calibration_gather->reset();
  runStepQueue("CLOSE_GRIPPER");
  gripper_state_ = "close";
}

void EngineerManual::leftSwitchDownFall()
{
  runStepQueue("HOME_ONE_STONE");
  runStepQueue("CLOSE_GRIPPER");
  drag_command_sender_->on();
  drag_state_ = "on";
  gripper_state_ = "off";
}

void EngineerManual::leftSwitchUpFall()
{
}

void EngineerManual::leftSwitchDownRise()
{
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
}

void EngineerManual::actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                        const rm_msgs::EngineerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->finish);
  change_flag_ = 1;
  ROS_INFO("Done %s", (prefix_ + root_).c_str());
  engineer_ui_.current_step_name += " done!";
  if (prefix_ + root_ == "TAKE_WHEN_TWO_STONE00" || prefix_ + root_ == "TAKE_WHEN_ONE_STONE00" ||
      prefix_ + root_ == "TAKE_WHEN_THREE_STONE00")
  {
    if (!stone_num_)
      stone_num_ = 0;
    else
      stone_num_ -= 1;
  }

  if (prefix_ + root_ == "STORE_WHEN_ZERO_STONE0" || prefix_ + root_ == "STORE_WHEN_ONE_STONE0" ||
      prefix_ + root_ == "STORE_WHEN_TWO_STONE0" || prefix_ + root_ == "SKY_BIG_ISLAND000" ||
      prefix_ + root_ == "GROUND_STONE00")
  {
    if (stone_num_ == 3)
      stone_num_ = 3;
    else
      stone_num_ += 1;
  }
  if (prefix_ + root_ == "THREE_STONE_SMALL_ISLAND00")
  {
    stone_num_ = 3;
  }
  operating_mode_ = MANUAL;
}

void EngineerManual::actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
{
  for (std::vector<int>::size_type i = 0; i < data->id.size(); ++i)
  {
    if (data->temperature[i] > max_temperature_)
    {
      max_temperature_ = data->temperature[i];
      max_temperature_joint_ = data->name[i];
      joint_temperature_ = max_temperature_joint_ + "=" + std::to_string(max_temperature_);
    }
  }
}

void EngineerManual::updateUiDate(std::string step_name, std::string reversal_state, std::string drag_state,
                                  uint8_t stone_num, std::string joint_temperature, std::string gripper_state)
{
  engineer_ui_.current_step_name = step_name;
  engineer_ui_.reversal_state = reversal_state;
  engineer_ui_.drag_state = drag_state;
  engineer_ui_.stone_num = stone_num;
  engineer_ui_.joint_temperature = joint_temperature;
  engineer_ui_.gripper_state = gripper_state;

  step_name_last_ = step_name;
  reversal_state_last_ = reversal_state;
  drag_state_last_ = drag_state;
  stone_num_last_ = stone_num;
  joint_temperature_last_ = joint_temperature;
  gripper_state_last_ = gripper_state;
}

bool EngineerManual::judgeUiChange(std::string step_name, std::string reversal_state, std::string drag_state,
                                   uint8_t stone_num, std::string joint_temperature, std::string gripper_state)
{
  if (step_name != step_name_last_ || reversal_state != reversal_state_last_ || drag_state != drag_state_last_ ||
      stone_num != stone_num_last_ || joint_temperature != joint_temperature_last_ ||
      gripper_state != gripper_state_last_)
  {
    updateUiDate(step_name, reversal_state, drag_state, stone_num, joint_temperature, gripper_state);
    return true;
  }
  else
    return false;
}

void EngineerManual::sendUi(std::string step_name, std::string reversal_state, std::string drag_state,
                            uint8_t stone_num, std::string joint_temperature, std::string gripper_state)
{
  if (judgeUiChange(step_name, reversal_state, drag_state, stone_num, joint_temperature, gripper_state))
    ui_send_.publish(engineer_ui_);
}

void EngineerManual::mouseLeftRelease()
{
  if (change_flag_)
  {
    root_ += "0";
    change_flag_ = 0;
    runStepQueue(prefix_ + root_);
    ROS_INFO("Finished %s", (prefix_ + root_).c_str());
  }
}

void EngineerManual::mouseRightRelease()
{
  runStepQueue(prefix_ + root_);
  ROS_INFO("Finished %s", (prefix_ + root_).c_str());
}
void EngineerManual::ctrlQPress()
{
  prefix_ = "LF_";
  root_ = "SMALL_ISLAND";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlWPress()
{
  prefix_ = "SKY_";
  root_ = "BIG_ISLAND";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlEPress()
{
  prefix_ = "RT_";
  root_ = "SMALL_ISLAND";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlRPress()
{
  calibration_gather->reset();
  engineer_ui_.current_step_name = "calibration";
  ROS_INFO("Calibrated");
}

void EngineerManual::ctrlAPress()
{
  prefix_ = "";
  root_ = "SMALL_ISLAND";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlSPress()
{
  prefix_ = "";
  root_ = "BIG_ISLAND";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlDPress()
{
  prefix_ = "";
  root_ = "GROUND_STONE0";
  runStepQueue(root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlFPress()
{
  prefix_ = "";
  root_ = "EXCHANGE_WAIT";
  runStepQueue(root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlGPress()
{
  switch (stone_num_)
  {
    case 0:
      root_ = "STORE_WHEN_ZERO_STONE0";
      break;
    case 1:
      root_ = "STORE_WHEN_ONE_STONE0";
      break;
    case 2:
      root_ = "STORE_WHEN_TWO_STONE0";
      break;
    case 3:
      root_ = "STORE_WHEN_TWO_STONE0";
      break;
  }
  runStepQueue(root_);
  prefix_ = "";
  ROS_INFO("STORE_STONE");
}

void EngineerManual::ctrlZPress()
{
  if (is_exchange_)
  {
    runStepQueue("GENERATE_EXCHANGE");
    action_client_.cancelAllGoals();
  }
}

void EngineerManual::ctrlZPressing()
{
  if (is_exchange_)
  {
    runStepQueue("GENERATE_EXCHANGE");
    action_client_.cancelAllGoals();
  }
}

void EngineerManual::ctrlZRelease()
{
  if (is_exchange_)
    runStepQueue("AUTO_EXCHANGER");
}

void EngineerManual::ctrlXPress()
{
  prefix_ = "THREE_STONE_";
  root_ = "SMALL_ISLAND";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlCPress()
{
  action_client_.cancelAllGoals();
  prefix_ = "";
  root_ = "";
  engineer_ui_.current_step_name = "cancel";
}

void EngineerManual::ctrlVPress()
{
  root_ = "";
  prefix_ = "";
  if (gripper_state_ == "open")
  {
    runStepQueue("CLOSE_GRIPPER");
    engineer_ui_.current_step_name = "CLOSE_GRIPPER";
    gripper_state_ = "close";
  }
  else if (gripper_state_ == "close")
  {
    runStepQueue("OPEN_GRIPPER");
    engineer_ui_.current_step_name = "OPEN_GRIPPER";
    gripper_state_ = "open";
  }
}

void EngineerManual::ctrlVRelease()
{
}

void EngineerManual::ctrlBPress()
{
  switch (stone_num_)
  {
    case 0:
      root_ = "HOME_ZERO_STONE";
      break;
    case 1:
      root_ = "HOME_ONE_STONE";
      break;
    case 2:
      root_ = "HOME_TWO_STONE";
      break;
    case 3:
      root_ = "HOME_THREE_STONE";
      break;
  }
  ROS_INFO("RUN_HOME");
  prefix_ = "";
  runStepQueue(root_);
}

void EngineerManual::qPressing()
{
  if (speed_change_mode_ == 1)
    vel_cmd_sender_->setAngularZVel(0.05);
  else
    vel_cmd_sender_->setAngularZVel(0.5);
}

void EngineerManual::qRelease()
{
  vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::ePressing()
{
  if (speed_change_mode_ == 1)
    vel_cmd_sender_->setAngularZVel(-0.05);
  else
    vel_cmd_sender_->setAngularZVel(-0.5);
}

void EngineerManual::eRelease()
{
  vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::zPressing()
{
  angular_z_scale_ = 0.1;
}

void EngineerManual::zRelease()
{
  angular_z_scale_ = 0.;
}

void EngineerManual::cPressing()
{
  angular_z_scale_ = -0.1;
}

void EngineerManual::cRelease()
{
  angular_z_scale_ = 0.;
}

void EngineerManual::rPress()
{
  if (!(stone_num_ == 3))
    stone_num_++;
  else
    stone_num_ = 0;
}

void EngineerManual::xPress()
{
  if (drag_state_ == "on")
  {
    drag_command_sender_->off();
    drag_state_ = "off";
  }
  else
  {
    drag_command_sender_->on();
    drag_state_ = "on";
  }
}
void EngineerManual::vPressing()
{
  // ROLL
  reversal_motion_ = 1;
  reversal_command_sender_->setGroupVel(0., 0., 0., 1., 0., 0.);
  reversal_state_ = "ROLL";
}

void EngineerManual::vRelease()
{
  // stop
  reversal_motion_ = 0;
  reversal_command_sender_->setZero();
  reversal_state_ = "STOP";
}

void EngineerManual::bPressing()
{
  // PITCH
  reversal_motion_ = 1;
  reversal_command_sender_->setGroupVel(0., 0., -0.3, 0., 1.5, 0.);
  reversal_state_ = "PITCH";
}

void EngineerManual::bRelease()
{
  // stop
  reversal_motion_ = 0;
  reversal_command_sender_->setZero();
  reversal_state_ = "STOP";
}

void EngineerManual::gPressing()
{
  // Z in
  reversal_motion_ = 1;
  reversal_command_sender_->setGroupVel(0., 0., -1., 0., 0., 0.);
  reversal_state_ = "Z IN";
}
void EngineerManual::gRelease()
{
  // stop
  reversal_motion_ = 0;
  reversal_command_sender_->setZero();
  reversal_state_ = "STOP";
}
void EngineerManual::fPressing()
{
  // Z out
  reversal_motion_ = 1;
  reversal_command_sender_->setGroupVel(0., 0., 1., 0., 0., 0.);
  reversal_state_ = "Z OUT";
}
void EngineerManual::fRelease()
{
  // stop
  reversal_command_sender_->setZero();
  reversal_state_ = "STOP";
}
void EngineerManual::shiftPressing()
{
  speed_change_mode_ = 0;
}
void EngineerManual::shiftRelease()
{
  speed_change_mode_ = 1;
}
void EngineerManual::shiftFPress()
{
  prefix_ = "";
  root_ = "EXCHANGE_GIMBAL";
  runStepQueue(root_);
  ROS_INFO("enter gimbal EXCHANGE_GIMBAL");
}
void EngineerManual::shiftRPress()
{
  prefix_ = "";
  root_ = "SKY_GIMBAL";
  runStepQueue(root_);
  ROS_INFO("enter gimbal SKY_GIMBAL");
}
void EngineerManual::shiftCPress()
{
  if (servo_mode_ == 1)
  {
    servo_mode_ = 0;
    engineer_ui_.current_step_name = "ENTER servo";
    ROS_INFO("EXIT SERVO");
  }
  else
  {
    servo_mode_ = 1;
    engineer_ui_.current_step_name = "exit SERVO";
    ROS_INFO("ENTER SERVO");
  }
  ROS_INFO("cancel all goal");
}
void EngineerManual::shiftZPress()
{
  prefix_ = "";
  root_ = "REVERSAL_GIMBAL";
  runStepQueue("REVERSAL_GIMBAL");
  ROS_INFO("enter gimbal REVERSAL_GIMBAL");
}
void EngineerManual::shiftVPress()
{
  // gimbal
  gimbal_mode_ = RATE;
  engineer_ui_.current_step_name = "gimbal MANUAL_VIEW";
  ROS_INFO("MANUAL_VIEW");
}

void EngineerManual::shiftVRelease()
{
  // gimbal
  gimbal_mode_ = DIRECT;
  ROS_INFO("DIRECT");
  engineer_ui_.current_step_name = "gimbal DIRECT";
}

void EngineerManual::shiftBPress()
{
  prefix_ = "";
  root_ = "BACK_GIMBAL";
  runStepQueue(root_);
  ROS_INFO("enter gimbal BACK_GIMBAL");
}

void EngineerManual::shiftBRelease()
{
}

void EngineerManual::shiftXPress()
{
  prefix_ = "";
  root_ = "GROUND_GIMBAL";
  runStepQueue(root_);
  ROS_INFO("enter gimbal GROUND_GIMBAL");
}

void EngineerManual::shiftGPress()
{
  switch (stone_num_)
  {
    case 0:
      root_ = "NO STONE!!";
      break;
    case 1:
      root_ = "TAKE_WHEN_ONE_STONE0";
      break;
    case 2:
      root_ = "TAKE_WHEN_TWO_STONE0";
      break;
    case 3:
      root_ = "TAKE_WHEN_THREE_STONE0";
      break;
  }
  prefix_ = "";
  runStepQueue(root_);

  ROS_INFO("TAKE_STONE");
}
}  // namespace rm_manual
