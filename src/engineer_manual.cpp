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
  engineer_ui_pub_ = nh.advertise<rm_msgs::EngineerUi>("/engineer_ui", 10);
  ROS_INFO("Waiting for middleware to start.");
  action_client_.waitForServer();
  ROS_INFO("Middleware started.");
  stone_num_sub_ = nh.subscribe<std_msgs::String>("/stone_num", 10, &EngineerManual::stoneNumCallback, this);
  gripper_state_sub_ = nh.subscribe<rm_msgs::GpioData>("/controllers/gpio_controller/gpio_states", 10,
                                                       &EngineerManual::gpioStateCallback, this);

  // Servo
  ros::NodeHandle nh_servo(nh, "servo");
  servo_command_sender_ = new rm_common::Vel3DCommandSender(nh_servo);
  servo_reset_caller_ = new rm_common::ServiceCallerBase<std_srvs::Empty>(nh_servo, "/servo_server/reset_servo_status");
  // Vel
  ros::NodeHandle chassis_nh(nh, "chassis");
  chassis_nh.param("fast_speed_scale", fast_speed_scale_, 1.0);
  chassis_nh.param("normal_speed_scale", normal_speed_scale_, 0.5);
  chassis_nh.param("low_speed_scale", low_speed_scale_, 0.3);
  chassis_nh.param("exchange_speed_scale", exchange_speed_scale_, 0.2);
  chassis_nh.param("fast_gyro_scale", fast_gyro_scale_, 0.5);
  chassis_nh.param("normal_gyro_scale", normal_gyro_scale_, 0.15);
  chassis_nh.param("low_gyro_scale", low_gyro_scale_, 0.05);
  chassis_nh.param("exchange_gyro_scale", exchange_gyro_scale_, 0.12);
  //  ros::NodeHandle nh_gimbal_lifter(nh, "gimbal_lifter");
  //  gimbal_lifter_command_sender_ = new rm_common::JointPointCommandSender(nh_gimbal_lifter, joint_state_);
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("pitch_calibration", rpc_value);
  pitch_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("calibration_gather", rpc_value);
  calibration_gather_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  // Key binding
  left_switch_up_event_.setFalling(boost::bind(&EngineerManual::leftSwitchUpFall, this));
  left_switch_up_event_.setRising(boost::bind(&EngineerManual::leftSwitchUpRise, this));
  left_switch_down_event_.setFalling(boost::bind(&EngineerManual::leftSwitchDownFall, this));
  left_switch_down_event_.setRising(boost::bind(&EngineerManual::leftSwitchDownRise, this));
  ctrl_a_event_.setRising(boost::bind(&EngineerManual::ctrlAPress, this));
  ctrl_b_event_.setRising(boost::bind(&EngineerManual::ctrlBPress, this));
  ctrl_c_event_.setRising(boost::bind(&EngineerManual::ctrlCPress, this));
  ctrl_d_event_.setRising(boost::bind(&EngineerManual::ctrlDPress, this));
  ctrl_e_event_.setRising(boost::bind(&EngineerManual::ctrlEPress, this));
  ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&EngineerManual::ctrlGPress, this));
  ctrl_q_event_.setRising(boost::bind(&EngineerManual::ctrlQPress, this));
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));
  ctrl_s_event_.setRising(boost::bind(&EngineerManual::ctrlSPress, this));
  ctrl_v_event_.setRising(boost::bind(&EngineerManual::ctrlVPress, this));
  ctrl_v_event_.setFalling(boost::bind(&EngineerManual::ctrlVRelease, this));
  ctrl_w_event_.setRising(boost::bind(&EngineerManual::ctrlWPress, this));
  ctrl_x_event_.setRising(boost::bind(&EngineerManual::ctrlXPress, this));
  ctrl_z_event_.setRising(boost::bind(&EngineerManual::ctrlZPress, this));
  b_event_.setActiveHigh(boost::bind(&EngineerManual::bPressing, this));
  b_event_.setFalling(boost::bind(&EngineerManual::bRelease, this));
  c_event_.setActiveHigh(boost::bind(&EngineerManual::cPressing, this));
  c_event_.setFalling(boost::bind(&EngineerManual::cRelease, this));
  e_event_.setActiveHigh(boost::bind(&EngineerManual::ePressing, this));
  e_event_.setFalling(boost::bind(&EngineerManual::eRelease, this));
  f_event_.setRising(boost::bind(&EngineerManual::fPress, this));
  f_event_.setFalling(boost::bind(&EngineerManual::fRelease, this));
  g_event_.setRising(boost::bind(&EngineerManual::gPress, this));
  g_event_.setFalling(boost::bind(&EngineerManual::gRelease, this));
  q_event_.setActiveHigh(boost::bind(&EngineerManual::qPressing, this));
  q_event_.setFalling(boost::bind(&EngineerManual::qRelease, this));
  r_event_.setRising(boost::bind(&EngineerManual::rPress, this));
  v_event_.setActiveHigh(boost::bind(&EngineerManual::vPressing, this));
  v_event_.setFalling(boost::bind(&EngineerManual::vRelease, this));
  x_event_.setRising(boost::bind(&EngineerManual::xPress, this));
  z_event_.setActiveHigh(boost::bind(&EngineerManual::zPressing, this));
  z_event_.setFalling(boost::bind(&EngineerManual::zRelease, this));
  shift_event_.setActiveHigh(boost::bind(&EngineerManual::shiftPressing, this));
  shift_event_.setFalling(boost::bind(&EngineerManual::shiftRelease, this));
  shift_b_event_.setRising(boost::bind(&EngineerManual::shiftBPress, this));
  shift_b_event_.setFalling(boost::bind(&EngineerManual::shiftBRelease, this));
  shift_c_event_.setRising(boost::bind(&EngineerManual::shiftCPress, this));
  shift_f_event_.setRising(boost::bind(&EngineerManual::shiftFPress, this));
  shift_g_event_.setRising(boost::bind(&EngineerManual::shiftGPress, this));
  shift_r_event_.setRising(boost::bind(&EngineerManual::shiftRPress, this));
  shift_r_event_.setFalling(boost::bind(&EngineerManual::shiftRRelease, this));
  shift_v_event_.setRising(boost::bind(&EngineerManual::shiftVPress, this));
  shift_v_event_.setFalling(boost::bind(&EngineerManual::shiftVRelease, this));
  shift_x_event_.setRising(boost::bind(&EngineerManual::shiftXPress, this));
  shift_z_event_.setRising(boost::bind(&EngineerManual::shiftZPress, this));

  mouse_left_event_.setFalling(boost::bind(&EngineerManual::mouseLeftRelease, this));
  mouse_right_event_.setFalling(boost::bind(&EngineerManual::mouseRightRelease, this));
}

void EngineerManual::run()
{
  ChassisGimbalManual::run();
  calibration_gather_->update(ros::Time::now());
  engineer_ui_pub_.publish(engineer_ui_);
}
void EngineerManual::changeSpeedMode(SpeedMode speed_mode)
{
  if (speed_mode == LOW)
  {
    speed_change_scale_ = low_speed_scale_;
    gyro_scale_ = low_gyro_scale_;
  }
  else if (speed_mode == NORMAL)
  {
    speed_change_scale_ = normal_speed_scale_;
    gyro_scale_ = normal_gyro_scale_;
  }
  else if (speed_mode == FAST)
  {
    speed_change_scale_ = fast_speed_scale_;
    gyro_scale_ = fast_gyro_scale_;
  }
  else if (speed_mode == EXCHANGE)
  {
    speed_change_scale_ = exchange_speed_scale_;
    gyro_scale_ = exchange_gyro_scale_;
  }
}
void EngineerManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::checkKeyboard(dbus_data);
  ctrl_a_event_.update(dbus_data->key_ctrl & dbus_data->key_a);
  ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b);
  ctrl_c_event_.update(dbus_data->key_ctrl & dbus_data->key_c);
  ctrl_d_event_.update(dbus_data->key_ctrl & dbus_data->key_d);
  ctrl_e_event_.update(dbus_data->key_ctrl & dbus_data->key_e);
  ctrl_f_event_.update(dbus_data->key_ctrl & dbus_data->key_f);
  ctrl_g_event_.update(dbus_data->key_ctrl & dbus_data->key_g);
  ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);
  ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
  ctrl_s_event_.update(dbus_data->key_ctrl & dbus_data->key_s);
  ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
  ctrl_w_event_.update(dbus_data->key_ctrl & dbus_data->key_w);
  ctrl_x_event_.update(dbus_data->key_ctrl & dbus_data->key_x);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);

  b_event_.update(dbus_data->key_b & !dbus_data->key_ctrl & !dbus_data->key_shift);
  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
  e_event_.update(dbus_data->key_e & !dbus_data->key_ctrl & !dbus_data->key_shift);
  f_event_.update(dbus_data->key_f & !dbus_data->key_ctrl & !dbus_data->key_shift);
  g_event_.update(dbus_data->key_g & !dbus_data->key_ctrl & !dbus_data->key_shift);
  q_event_.update(dbus_data->key_q & !dbus_data->key_ctrl & !dbus_data->key_shift);
  r_event_.update(dbus_data->key_r & !dbus_data->key_ctrl & !dbus_data->key_shift);
  v_event_.update(dbus_data->key_v & !dbus_data->key_ctrl & !dbus_data->key_shift);
  x_event_.update(dbus_data->key_x & !dbus_data->key_ctrl & !dbus_data->key_shift);
  z_event_.update(dbus_data->key_z & !dbus_data->key_ctrl & !dbus_data->key_shift);

  shift_event_.update(dbus_data->key_shift & !dbus_data->key_ctrl);
  shift_b_event_.update(dbus_data->key_shift & dbus_data->key_b);
  shift_c_event_.update(dbus_data->key_shift & dbus_data->key_c);
  shift_e_event_.update(dbus_data->key_shift & dbus_data->key_e);
  shift_g_event_.update(dbus_data->key_shift & dbus_data->key_g);
  shift_q_event_.update(dbus_data->key_shift & dbus_data->key_q);
  shift_r_event_.update(dbus_data->key_shift & dbus_data->key_r);
  shift_v_event_.update(dbus_data->key_shift & dbus_data->key_v);
  shift_x_event_.update(dbus_data->key_shift & dbus_data->key_x);
  shift_z_event_.update(dbus_data->key_shift & dbus_data->key_z);

  mouse_left_event_.update(dbus_data->p_l);
  mouse_right_event_.update(dbus_data->p_r);

  c_event_.update(dbus_data->key_c & !dbus_data->key_ctrl & !dbus_data->key_shift);
}
void EngineerManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updateRc(dbus_data);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
  vel_cmd_sender_->setAngularZVel(dbus_data->wheel);
  vel_cmd_sender_->setLinearXVel(dbus_data->ch_r_y);
  vel_cmd_sender_->setLinearYVel(-dbus_data->ch_r_x);

  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(dbus_data->s_l == rm_msgs::DbusData::DOWN);
}
void EngineerManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  checkKeyboard(dbus_data);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  if (servo_mode_ == JOINT)
    vel_cmd_sender_->setAngularZVel(-dbus_data->m_x * gimbal_scale_);
}
void EngineerManual::updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  servo_command_sender_->setLinearVel(-dbus_data->wheel, -dbus_data->ch_l_x, -dbus_data->ch_l_y);
  servo_command_sender_->setAngularVel(-angular_z_scale_, dbus_data->ch_r_y, -dbus_data->ch_r_x);
}
void EngineerManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
  if (servo_mode_ == SERVO)
    updateServo(data);
}
void EngineerManual::stoneNumCallback(const std_msgs::String::ConstPtr& data)
{
  if (data->data == "-1" && stone_num_ > 0)
    stone_num_--;
  else if (data->data == "+1" && stone_num_ < 2)
    stone_num_++;
}
void EngineerManual::gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data)
{
  gpio_state_.gpio_state = data->gpio_state;
  if (gpio_state_.gpio_state[0])
  {
    gripper_state_ = "open";
  }
  else
  {
    gripper_state_ = "close";
  }
}
void EngineerManual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendChassisCommand(time, false);
    vel_cmd_sender_->sendCommand(time);
  }
  if (servo_mode_ == SERVO)
  {
    changeSpeedMode(EXCHANGE);
    servo_command_sender_->sendCommand(time);
    if (gimbal_mode_ == RATE)
      gimbal_cmd_sender_->sendCommand(time);
  }
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
  ROS_INFO("Done %s", (prefix_ + root_).c_str());
  change_flag_ = true;
  ROS_INFO("%i", result->finish);
  operating_mode_ = MANUAL;
  if (prefix_ + root_ == "EXCHANGE_POS" || prefix_ + root_ == "GET_DOWN_STONE_BIN" ||
      prefix_ + root_ == "GET_UP_STONE_BIN")
  {
    enterServo();
    changeSpeedMode(EXCHANGE);
  }
  if (prefix_ == "HOME_")
  {
    initMode();
    changeSpeedMode(NORMAL);
  }
}

void EngineerManual::enterServo()
{
  servo_mode_ = SERVO;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(EXCHANGE);
  servo_reset_caller_->callService();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
  chassis_cmd_sender_->getMsg()->command_source_frame = "link4";
}
void EngineerManual::initMode()
{
  servo_mode_ = JOINT;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(NORMAL);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
}

void EngineerManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}
void EngineerManual::gimbalOutputOn()
{
  ChassisGimbalManual::gimbalOutputOn();
  pitch_calibration_->reset();
  ROS_INFO("pitch calibrated");
}
void EngineerManual::chassisOutputOn()
{
  if (operating_mode_ == MIDDLEWARE)
    action_client_.cancelAllGoals();
}

//-------------------controller input-------------------
void EngineerManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  gimbal_mode_ = DIRECT;
  servo_mode_ = JOINT;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
void EngineerManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  servo_mode_ = JOINT;
  gimbal_mode_ = RATE;
  gimbal_cmd_sender_->setZero();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
void EngineerManual::rightSwitchDownRise()
{
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  servo_mode_ = SERVO;
  gimbal_mode_ = RATE;
  servo_reset_caller_->callService();
  action_client_.cancelAllGoals();
  ROS_INFO_STREAM("servo_mode");
}

void EngineerManual::leftSwitchUpRise()
{
  prefix_ = "";
  root_ = "CALIBRATION";
  calibration_gather_->reset();
  ROS_INFO_STREAM("START CALIBRATE");
}
void EngineerManual::leftSwitchUpFall()
{
  runStepQueue("HOME_ZERO_STONE");
  runStepQueue("CLOSE_GRIPPER");
}

void EngineerManual::leftSwitchDownRise()
{
  if (gripper_state_ == "close")
  {
    runStepQueue("OPEN_GRIPPER");
  }
  else
  {
    runStepQueue("CLOSE_GRIPPER");
  }
}
void EngineerManual::leftSwitchDownFall()
{
}
// mouse input
void EngineerManual::mouseLeftRelease()
{
  if (change_flag_)
  {
    root_ += "0";
    change_flag_ = false;
    runStepQueue(prefix_ + root_);
    ROS_INFO("Finished %s", (prefix_ + root_).c_str());
  }
}

void EngineerManual::mouseRightRelease()
{
  runStepQueue(prefix_ + root_);
  ROS_INFO("Finished %s", (prefix_ + root_).c_str());
}
//--------------------- keyboard input ------------------------

//------------------------- ctrl ------------------------------
void EngineerManual::ctrlAPress()
{
  prefix_ = "ONE_STONE_";
  root_ = "SMALL_ISLAND";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlBPress()
{
  prefix_ = "HOME_";
  switch (stone_num_)
  {
    case 0:
      root_ = "ZERO_STONE";
      break;
    case 1:
      root_ = "ONE_STONE";
      break;
    case 2:
      root_ = "TWO_STONE";
      break;
  }
  runStepQueue(prefix_ + root_);
  ROS_INFO_STREAM("RUN_HOME" << stone_num_);
}

void EngineerManual::ctrlCPress()
{
  action_client_.cancelAllGoals();
  ROS_INFO("cancel all goal");
}

void EngineerManual::ctrlDPress()
{
  prefix_ = "TWO_STONE_";
  root_ = "SMALL_ISLAND";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlEPress()
{
}

void EngineerManual::ctrlFPress()
{
  prefix_ = "";
  root_ = "EXCHANGE_POS";
  runStepQueue(root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlGPress()
{
  //  switch (stone_num_)
  //  {
  //    case 0:
  //      root_ = "STORE_WHEN_ZERO_STONE";
  //      stone_num_ = 1;
  //      break;
  //    case 1:
  //      root_ = "DOWN_STONE_FROM_BIN";
  //      stone_num_ = 2;
  //      break;
  //  }
  //  runStepQueue(root_);
  //  prefix_ = "";

  //  ROS_INFO("STORE_STONE");
}

void EngineerManual::ctrlQPress()
{
}

void EngineerManual::ctrlRPress()
{
  prefix_ = "";
  root_ = "CALIBRATION";
  servo_mode_ = JOINT;
  calibration_gather_->reset();
  runStepQueue("CLOSE_GRIPPER");
  ROS_INFO_STREAM("START CALIBRATE");
}

void EngineerManual::ctrlSPress()
{
  prefix_ = "MID_";
  root_ = "BIG_ISLAND";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlVPress()
{
}
void EngineerManual::ctrlVRelease()
{
}

void EngineerManual::ctrlWPress()
{
  prefix_ = "SIDE_";
  root_ = "BIG_ISLAND";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlXPress()
{
}

void EngineerManual::ctrlZPress()
{
}
//-------------------------- keys ------------------------------
void EngineerManual::bPressing()
{
}
void EngineerManual::bRelease()
{
}

void EngineerManual::cPressing()
{
  angular_z_scale_ = 0.5;
  ROS_INFO_STREAM("angular_z_scale is 0.5");
}
void EngineerManual::cRelease()
{
  angular_z_scale_ = 0.;
}

void EngineerManual::ePressing()
{
  if (servo_mode_ == SERVO)
    vel_cmd_sender_->setAngularZVel(-gyro_scale_);
}
void EngineerManual::eRelease()
{
  if (servo_mode_ == SERVO)
    vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::fPress()
{
  switch (gimbal_height_)
  {
    case 0:
      runStepQueue("GIMBAL_DOWN");
      gimbal_height_ = 1;
      break;
    case 1:
      runStepQueue("GIMBAL_MID");
      gimbal_height_ = 2;
      break;
    case 2:
      runStepQueue("GIMBAL_TALL");
      gimbal_height_ = 0;
      break;
  }
}
void EngineerManual::fRelease()
{
}

void EngineerManual::gPress()
{
}
void EngineerManual::gRelease()
{
}

void EngineerManual::qPressing()
{
  if (servo_mode_ == SERVO)
    vel_cmd_sender_->setAngularZVel(gyro_scale_);
}
void EngineerManual::qRelease()
{
  if (servo_mode_ == SERVO)
    vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::rPress()
{
  if (stone_num_ < 2)
    stone_num_++;
  else
    stone_num_ = 0;
  ROS_INFO_STREAM("Stone num is: " << stone_num_);
}

void EngineerManual::vPressing()
{
}
void EngineerManual::vRelease()
{
}

void EngineerManual::xPress()
{
}

void EngineerManual::zPressing()
{
  angular_z_scale_ = -0.5;
  ROS_INFO_STREAM("angular_z_scale is -0.5");
}
void EngineerManual::zRelease()
{
  angular_z_scale_ = 0.;
}
//----------------------------- shift ----------------------------
void EngineerManual::shiftPressing()
{
  changeSpeedMode(FAST);
  ROS_DEBUG_STREAM("speed mode is fast");
}
void EngineerManual::shiftRelease()
{
  changeSpeedMode(NORMAL);
  ROS_DEBUG_STREAM("speed mode is normal");
}

void EngineerManual::shiftBPress()
{
}
void EngineerManual::shiftBRelease()
{
}

void EngineerManual::shiftCPress()
{
  action_client_.cancelAllGoals();
  if (servo_mode_ == SERVO)
  {
    initMode();
    ROS_INFO("EXIT SERVO");
  }
  else
  {
    enterServo();
    ROS_INFO("ENTER SERVO");
  }
  ROS_INFO("cancel all goal");
}

void EngineerManual::shiftFPress()
{
}

void EngineerManual::shiftGPress()
{
  switch (stone_num_)
  {
    case 0:
      root_ = "NO STONE!!";
      stone_num_ = 0;
      break;
    case 1:
      root_ = "GET_DOWN_STONE_BIN";
      break;
    case 2:
      root_ = "GET_UP_STONE_BIN";
      break;
  }
  runStepQueue(root_);
  prefix_ = "";

  ROS_INFO("TAKE_STONE");
}

void EngineerManual::shiftRPress()
{
}
void EngineerManual::shiftRRelease()
{
}

void EngineerManual::shiftVPress()
{
  if (gripper_state_ == "close")
  {
    runStepQueue("OPEN_GRIPPER");
  }
  else
  {
    runStepQueue("CLOSE_GRIPPER");
  }
}
void EngineerManual::shiftVRelease()
{
}

void EngineerManual::shiftXPress()
{
}

void EngineerManual::shiftZPress()
{
}

}  // namespace rm_manual
