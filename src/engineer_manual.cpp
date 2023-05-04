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
  // Vel
  ros::NodeHandle vel_nh(nh, "vel");
  if (!vel_nh.getParam("gyro_scale", gyro_scale_))
    gyro_scale_ = 0.5;
  if (!vel_nh.getParam("gyro_low_scale", gyro_low_scale_))
    gyro_low_scale_ = 0.05;
  // Ui
  exchange_sub_ = nh.subscribe<rm_msgs::ExchangerMsg>("/pnp_publisher", 10, &EngineerManual::exchangeCallback, this);
  engineer_ui_pub_ = nh.advertise<rm_msgs::EngineerUi>("/engineer_ui", 10);
  stone_num_sub_ = nh.subscribe<std_msgs::String>("/stone_num", 10, &EngineerManual::stoneNumCallback, this);
  // Drag
  ros::NodeHandle nh_drag(nh, "drag");
  drag_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_drag);
  // Joint7
  ros::NodeHandle nh_joint7(nh, "joint7");
  joint7_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_joint7);
  // Reversal
  ros::NodeHandle nh_reversal(nh, "reversal");
  reversal_command_sender_ = new rm_common::MultiDofCommandSender(nh_reversal);
  // Servo
  ros::NodeHandle nh_servo(nh, "servo");
  servo_command_sender_ = new rm_common::Vel3DCommandSender(nh_servo);
  servo_reset_caller_ = new rm_common::ServiceCallerBase<std_srvs::Empty>(nh_servo, "/servo_server/reset_servo_status");
  // Gripper State
  gripper_state_sub_ = nh.subscribe<rm_msgs::GpioData>("/controllers/gpio_controller/gpio_states", 10,
                                                       &EngineerManual::gpioStateCallback, this);
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("calibration_gather", rpc_value);
  calibration_gather_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("joint5_calibration", rpc_value);
  joint5_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
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
  g_event_.setRising(boost::bind(&EngineerManual::gPress, this));
  g_event_.setFalling(boost::bind(&EngineerManual::gRelease, this));
  b_event_.setActiveHigh(boost::bind(&EngineerManual::bPressing, this));
  b_event_.setFalling(boost::bind(&EngineerManual::bRelease, this));
  f_event_.setRising(boost::bind(&EngineerManual::fPress, this));
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
  calibration_gather_->update(ros::Time::now());
  sendUi();
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

void EngineerManual::stoneNumCallback(const std_msgs::String ::ConstPtr& data)
{
  std::cout << stone_num_ << std::endl;
  if (data->data == "-1")
    stone_num_ -= 1;
  else if (data->data == "+1")
    stone_num_ += 1;
  else if (data->data == "+3")
    stone_num_ += 3;
  if (stone_num_ >= 4)
    stone_num_ = 3;
  else if (stone_num_ <= -1)
    stone_num_ = 0;
}
void EngineerManual::gpioStateCallback(const rm_msgs::GpioData ::ConstPtr& data)
{
  gpio_state_.gpio_state = data->gpio_state;
  if (!gpio_state_.gpio_state[0])
    gripper_state_ = "open";
  else
    gripper_state_ = "close";
}
void EngineerManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updateRc(dbus_data);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  vel_cmd_sender_->setAngularZVel(dbus_data->wheel);
  vel_cmd_sender_->setLinearXVel(dbus_data->ch_r_y);
  vel_cmd_sender_->setLinearYVel(-dbus_data->ch_r_x);

  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  left_switch_down_event_.update(dbus_data->s_l == rm_msgs::DbusData::DOWN);
}

void EngineerManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::updatePc(dbus_data);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  if (!reversal_motion_ && servo_mode_ == JOINT)
    reversal_command_sender_->setGroupValue(0., 0., 5 * dbus_data->ch_r_y, 5 * dbus_data->ch_l_x, 5 * dbus_data->ch_l_y,
                                            0.);
  if (is_auxiliary_camera_)
  {
    //    chassis_cmd_sender_->getMsg()->follow_source_frame = base_link;
    //    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  else
  {
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  }
}

void EngineerManual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendChassisCommand(time, false);
    vel_cmd_sender_->sendCommand(time);
    reversal_command_sender_->sendCommand(time);
    drag_command_sender_->sendCommand(time);
  }
  if (servo_mode_ == SERVO)
    servo_command_sender_->sendCommand(time);
  if (gimbal_mode_ == RATE)
    gimbal_cmd_sender_->sendCommand(time);
  // judgeJoint7(time);
}

void EngineerManual::updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  servo_command_sender_->setLinearVel(dbus_data->ch_l_y, -dbus_data->ch_l_x, -dbus_data->wheel);
  servo_command_sender_->setAngularVel(dbus_data->ch_r_x, dbus_data->ch_r_y, angular_z_scale_);
  ChassisGimbalManual::updatePc(dbus_data);
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
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
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
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  gimbal_mode_ = DIRECT;
  servo_mode_ = JOINT;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::leftSwitchUpRise()
{
  prefix_ = "";
  root_ = "CALIBRATION";
  calibration_gather_->reset();
  EngineerManual::ctrlVPress();
  ROS_INFO_STREAM("START CALIBRATE");
}

void EngineerManual::leftSwitchDownFall()
{
  //  runStepQueue("HOME_ONE_STONE");
  drag_command_sender_->on();
  drag_state_ = "on";
}

void EngineerManual::leftSwitchUpFall()
{
  joint5_calibration_->reset();
}

void EngineerManual::leftSwitchDownRise()
{
}

void EngineerManual::runStepQueue(const std::string& step_queue_name)
{
  reversal_motion_ = true;
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
  operating_mode_ = MANUAL;
  reversal_motion_ = false;
  change_flag_ = true;
}

void EngineerManual::actuatorStateCallback(const rm_msgs::ActuatorState::ConstPtr& data)
{
  max_temperature_ = data->temperature[0];
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

void EngineerManual::sendUi()
{
  engineer_ui_.current_step_name = prefix_ + root_;
  engineer_ui_.reversal_state = reversal_state_;
  engineer_ui_.drag_state = drag_state_;
  engineer_ui_.stone_num = stone_num_;
  engineer_ui_.joint_temperature = joint_temperature_;
  engineer_ui_.gripper_state = gripper_state_;
  engineer_ui_pub_.publish(engineer_ui_);
}

void EngineerManual::judgeJoint7(const ros::Time& time)
{
  if (prefix_ + root_ == "GROUND_STONE0" || prefix_ == "EXCHANGE_" || prefix_ + root_ == "GROUND_STONE00")
  {
    joint7_command_sender_->off();
  }
  else
  {
    joint7_command_sender_->on();
  }
  joint7_command_sender_->sendCommand(time);
}

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
  prefix_ = "";
  root_ = "CALIBRATION";
  servo_mode_ = JOINT;
  calibration_gather_->reset();
  joint5_calibration_->reset();
  runStepQueue("CLOSE_GRIPPER");
  ROS_INFO_STREAM("START CALIBRATE");
}

void EngineerManual::ctrlAPress()
{
  prefix_ = "";
  root_ = "SMALL_ISLAND";
  runStepQueue("SMALL_ISLAND");
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
  prefix_ = "GROUND_";
  root_ = "STONE0";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlFPress()
{
  prefix_ = "EXCHANGE_";
  root_ = "WAIT";
  runStepQueue(prefix_ + root_);
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
  prefix_ = "";
  runStepQueue(prefix_ + root_);
  ROS_INFO("STORE_STONE");
}

void EngineerManual::ctrlZPress()
{
  if (is_exchange_)
  {
    prefix_ = "EXCHANGE_";
    root_ = "GENERATE";
    runStepQueue(prefix_ + root_);
    action_client_.cancelAllGoals();
  }
}

void EngineerManual::ctrlZPressing()
{
  if (is_exchange_)
  {
    prefix_ = "EXCHANGE_";
    root_ = "GENERATE";
    runStepQueue(prefix_ + root_);
    action_client_.cancelAllGoals();
  }
}

void EngineerManual::ctrlZRelease()
{
  if (is_exchange_)
  {
    prefix_ = "EXCHANGE_";
    root_ = "AUTO";
    runStepQueue(prefix_ + root_);
  }
}

void EngineerManual::ctrlXPress()
{
  //  prefix_ = "THREE_STONE_";
  //  root_ = "SMALL_ISLAND";
  prefix_ = "";
  root_ = "NEW_THREE_STONE_SMALL_ISLAND";
  runStepQueue("THREE_STONE_SMALL_ISLAND");
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlCPress()
{
  runStepQueue("DELETE_SCENE");
  action_client_.cancelAllGoals();
  prefix_ = "";
  root_ = "CANCEL GOALS";
}

void EngineerManual::ctrlVPress()
{
  if (gripper_state_ == "open")
    runStepQueue("CLOSE_GRIPPER");
  else
    runStepQueue("OPEN_GRIPPER");
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
  runStepQueue(prefix_ + root_);
}

void EngineerManual::qPressing()
{
  vel_cmd_sender_->setAngularZVel(speed_change_mode_ ? gyro_low_scale_ : gyro_scale_);
}

void EngineerManual::qRelease()
{
  vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::ePressing()
{
  vel_cmd_sender_->setAngularZVel(speed_change_mode_ ? -gyro_low_scale_ : -gyro_scale_);
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
  if (prefix_ != "ENGINEER_")
  {
    prefix_ = "ENGINEER_";
    root_ = "DRAG_CAR";
  }
  else
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
}

void EngineerManual::bPressing()
{
  // ROLL
  reversal_motion_ = true;
  reversal_command_sender_->setGroupValue(0., 0., 0., 1., 0., 0.);
  reversal_state_ = "ROLL";
}

void EngineerManual::vRelease()
{
  // stop
  reversal_motion_ = false;
  reversal_command_sender_->setZero();
  reversal_state_ = "STOP";
}

void EngineerManual::gPress()
{
  // PITCH
  runStepQueue("PITCH_PI_2");
  // reversal_command_sender_->setGroupVel(0., 0., -0.3, 0., 1.5, 0.);
  reversal_state_ = "PITCH";
}

void EngineerManual::gRelease()
{
  // stop
  runStepQueue("POSITION_STOP");
  reversal_state_ = "STOP";
}
void EngineerManual::bRelease()
{
  // stop
  reversal_motion_ = false;
  reversal_command_sender_->setZero();
  reversal_state_ = "STOP";
}

void EngineerManual::vPressing()
{
  // Z in
  reversal_motion_ = true;
  reversal_command_sender_->setGroupValue(0., 0., -1., 0., 0., 0.);
  reversal_state_ = "Z IN";
}
void EngineerManual::fPress()
{
  // Z out
  runStepQueue("Z_REVERSAL_OUT");
  reversal_state_ = "Z OUT";
}
void EngineerManual::fRelease()
{
  // stop
  runStepQueue("POSITION_STOP");
  reversal_state_ = "STOP";
}
void EngineerManual::shiftPressing()
{
  speed_change_mode_ = false;
}
void EngineerManual::shiftRelease()
{
  speed_change_mode_ = true;
}
void EngineerManual::shiftFPress()
{
  //  prefix_ = "";
  //  root_ = "EXCHANGE_GIMBAL";
  runStepQueue("EXCHANGE_GIMBAL");
  ROS_INFO("enter gimbal EXCHANGE_GIMBAL");
}
void EngineerManual::shiftRPress()
{
  //  prefix_ = "";
  //  root_ = "SKY_GIMBAL";
  //  runStepQueue(prefix_ + root_);
  runStepQueue("SKY_GIMBAL");
  ROS_INFO("enter gimbal SKY_GIMBAL");
}
void EngineerManual::shiftCPress()
{
  if (is_joint7_up_)
  {
    runStepQueue("JOINT7_DOWN");
    is_joint7_up_ = false;
  }
  else
  {
    runStepQueue("JOINT7_UP");
    is_joint7_up_ = true;
  }
}
void EngineerManual::shiftZPress()
{
  //  prefix_ = "";
  //  root_ = "ISLAND_GIMBAL";
  //  runStepQueue(prefix_ + root_);
  runStepQueue("ISLAND_GIMBAL");
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
  //  prefix_ = "";
  //  root_ = "SIDE_GIMBAL";
  //  runStepQueue(prefix_ + root_);
  runStepQueue("SIDE_GIMBAL");
  ROS_INFO("enter gimbal BACK_GIMBAL");
}

void EngineerManual::shiftBRelease()
{
}

void EngineerManual::shiftXPress()
{
  runStepQueue("GROUND_GIMBAL");
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
  runStepQueue(prefix_ + root_);

  ROS_INFO("TAKE_STONE");
}
}  // namespace rm_manual
