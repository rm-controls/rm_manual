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
  // Auto Exchange
  XmlRpc::XmlRpcValue auto_exchange_value;
  nh.getParam("auto_exchange", auto_exchange_value);
  ros::NodeHandle nh_auto_exchange(nh, "auto_exchange");
//  auto_exchange_ = new auto_exchange::AutoExchange(auto_exchange_value, tf_buffer_, nh_auto_exchange);
  // Pub
//  exchanger_update_pub_ = nh.advertise<std_msgs::Bool>("/is_update_exchanger", 1);
  // Sub
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
  chassis_nh.param("exchange_speed_scale", exchange_speed_scale_, 0.3);
  chassis_nh.param("fast_gyro_scale", fast_gyro_scale_, 0.5);
  chassis_nh.param("normal_gyro_scale", normal_gyro_scale_, 0.15);
  chassis_nh.param("low_gyro_scale", low_gyro_scale_, 0.05);
  chassis_nh.param("exchange_gyro_scale", exchange_gyro_scale_, 0.12);
  //Extend arm
  ros::NodeHandle nh_extend_arm_a(nh, "extend_arm_a");
  ros::NodeHandle nh_extend_arm_b(nh, "extend_arm_b");
  extend_arm_a_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_extend_arm_a);
  extend_arm_b_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_extend_arm_b);
  //Ore bin
  ros::NodeHandle nh_ore_bin_lifter(nh, "ore_bin_lifter");
  ros::NodeHandle nh_ore_bin_rotator(nh, "ore_bin_rotator");
  ore_bin_lifter_command_sender_ = new rm_common::JointPointCommandSender(nh_ore_bin_lifter, joint_state_);
  ore_bin_rotate_command_sender_ = new rm_common::JointPointCommandSender(nh_ore_bin_rotator, joint_state_);
  ros::NodeHandle nh_gimbal_lifter(nh, "gimbal_lifter");
  gimbal_lifter_command_sender_ = new rm_common::JointPointCommandSender(nh_gimbal_lifter, joint_state_);
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
  x_event_.setRising(boost::bind(&EngineerManual::xPress,this));
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
}
void EngineerManual::updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
//  if (auto_exchange_->union_move_->getEnterFlag() != true && !auto_exchange_->getFinishFlag())
//  {
     servo_command_sender_->setLinearVel(dbus_data->wheel, -dbus_data->ch_l_x, dbus_data->ch_l_y);
     servo_command_sender_->setAngularVel(-angular_z_scale_, dbus_data->ch_r_y, dbus_data->ch_r_x);
//  }
//  else
//  {
//    if (!auto_exchange_->union_move_->getFinishFlag())
//    {
//      std::vector<double> servo_scales;
//      servo_scales.resize(3, 0.);
//      servo_scales = auto_exchange_->union_move_->getServoScale();
//      servo_command_sender_->setLinearVel(servo_scales[0], servo_scales[1], servo_scales[2]);
//    }
//    else
//      servo_command_sender_->setZero();
//  }
  ChassisGimbalManual::updatePc(dbus_data);
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
  ROS_INFO_STREAM("stone num: "<< stone_num_);
  if (data->data == "-1" && stone_num_ != 0)
    stone_num_--;
  else
    stone_num_++;
  if (stone_num_ >= 3)
    stone_num_--;
  engineer_ui_.stone_num = std::to_string(stone_num_);
}
void EngineerManual::gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data)
{
  gpio_state_.gpio_state = data->gpio_state;
  if (gpio_state_.gpio_state[0])
  {
    gripper_state_ = "open";
    engineer_ui_.gripper_state = "open";
  }
  else
  {
    gripper_state_ = "close";
    engineer_ui_.gripper_state = "close";
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
  engineer_ui_.stone_num = stone_num_;
  ROS_INFO("%i", result->finish);

  operating_mode_ = MANUAL;
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
  shiftVPress();
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  servo_mode_ = SERVO;
  gimbal_mode_ = RATE;
  servo_reset_caller_->callService();
  action_client_.cancelAllGoals();
  ROS_INFO_STREAM(servo_mode_);
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
  runStepQueue("HOME_ZERO_STONE");
}
void EngineerManual::leftSwitchDownFall()
{
  runStepQueue("HOME_ZERO_STONE");
  drag_state_ = "off";
  engineer_ui_.drag_state = "off";
}
//mouse input
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
//keyboard input
//------------------------- ctrl ------------------------------
void EngineerManual::ctrlAPress()
{
  prefix_ = "MID_";
  root_ = "SMALL_ISLAND";
  runStepQueue(prefix_+root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlBPress()
{
  switch (stone_num_)
  {
    case 0:
      root_ = "_ZERO_STONE";
      break;
    case 1:
      root_ = "_ONE_STONE";
      break;
    case 2:
      root_ = "_TWO_STONE";
      break;
  }
  ROS_INFO("RUN_HOME");

  prefix_ = "HOME";
  runStepQueue(prefix_ + root_);
}

void EngineerManual::ctrlCPress()
{
  action_client_.cancelAllGoals();
}

void EngineerManual::ctrlDPress()
{
  prefix_ = "R_";
  root_ = "SMALL_ISLAND";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlEPress()
{
  prefix_ = "L_";
  root_ = "SMALL_ISLAND";
  runStepQueue(prefix_ + root_);
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
      root_ = "STORE_WHEN_ZERO_STONE";
      stone_num_ = 1;
      break;
    case 1:
      root_ = "STORE_WHEN_ONE_STONE";
      stone_num_ = 2;
      break;
  }
  runStepQueue(root_);
  prefix_ = "";

  ROS_INFO("STORE_STONE");
}

void EngineerManual::ctrlQPress()
{
  prefix_ = "L_";
  root_ = "SMALL_ISLAND";
  ROS_INFO("%s", (prefix_ + root_).c_str());
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
  prefix_ = "";
  root_ = "BIG_ISLAND";

  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlVPress()
{
  if (state_)
  {
    runStepQueue("CLOSE_GRIPPER");
    state_ = false;
  }
  else if (!state_)
  {
    runStepQueue("OPEN_GRIPPER");
    state_ = true;
  }
}
void EngineerManual::ctrlVRelease()
{
}

void EngineerManual::ctrlWPress()
{
  prefix_ = "SKY_";
  root_ = "BIG_ISLAND";
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
  angular_z_scale_ = -0.1;
}
void EngineerManual::cRelease()
{
  angular_z_scale_ = 0.;
}

void EngineerManual::ePressing()
{
  vel_cmd_sender_->setAngularZVel(-gyro_scale_);
}
void EngineerManual::eRelease()
{
  vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::fPress()
{
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
  vel_cmd_sender_->setAngularZVel(gyro_scale_);
}
void EngineerManual::qRelease()
{
  vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::rPress()
{
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
  angular_z_scale_ = 0.1;
}
void EngineerManual::zRelease()
{
  angular_z_scale_ = 0.;
}
//----------------------------- shift ----------------------------
void EngineerManual::shiftPressing()
{
  changeSpeedMode(FAST);
}
void EngineerManual::shiftRelease()
{
  changeSpeedMode(NORMAL);
}

void EngineerManual::shiftBPress()
{
  runStepQueue("TEMP_GIMBAL");
  ROS_INFO("enter gimbal BACK_GIMBAL");
}
void EngineerManual::shiftBRelease()
{
  runStepQueue("BACK_GIMBAL");
}

void EngineerManual::shiftCPress()
{
  if (servo_mode_)
  {
    servo_mode_ = false;

    ROS_INFO("EXIT SERVO");
  }
  else
  {
    servo_mode_ = true;

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
      root_ = "TAKE_WHEN_ONE_STONE";
      stone_num_ = 0;
      break;
    case 2:
      root_ = "TAKE_WHEN_TWO_STONE";
      stone_num_ = 1;
      break;
  }
  runStepQueue(root_);
  prefix_ = "";

  ROS_INFO("TAKE_STONE");
}

void EngineerManual::shiftRPress()
{
  runStepQueue("SKY_GIMBAL");
  ROS_INFO("enter gimbal SKY_GIMBAL");
}
void EngineerManual::shiftRRelease()
{
}

void EngineerManual::shiftVPress()
{
  if (gripper_state_ == "close")
  {
    runStepQueue("CLOSE_GRIPPER");
    engineer_ui_.gripper_state = "open";
  }
  else
  {
    runStepQueue("OPEN_GRIPPER");
    engineer_ui_.gripper_state = "close";
  }
}
void EngineerManual::shiftVRelease()
{
  // gimbal
  gimbal_mode_ = DIRECT;
  ROS_INFO("DIRECT");
}

void EngineerManual::shiftXPress()
{
  runStepQueue("GROUND_GIMBAL");
  ROS_INFO("enter gimbal GROUND_GIMBAL");
}

void EngineerManual::shiftZPress()
{

}

}  // namespace rm_manual
