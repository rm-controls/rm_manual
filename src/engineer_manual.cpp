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
  auto_exchange_ = new auto_exchange::AutoExchange(auto_exchange_value, tf_buffer_, nh_auto_exchange);
  // Pub
  exchanger_update_pub_ = nh.advertise<std_msgs::Bool>("/is_update_exchanger", 1);
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
  if (!chassis_nh.getParam("fast_speed_scale", fast_speed_scale_))
    fast_speed_scale_ = 1;
  if (!chassis_nh.getParam("normal_speed_scale", normal_speed_scale_))
    normal_speed_scale_ = 0.5;
  if (!chassis_nh.getParam("low_speed_scale", low_speed_scale_))
    low_speed_scale_ = 0.30;
  if (!chassis_nh.getParam("exchange_speed_scale", exchange_speed_scale_))
    exchange_speed_scale_ = 0.30;
  if (!chassis_nh.getParam("fast_gyro_scale", fast_gyro_scale_))
    fast_gyro_scale_ = 0.5;
  if (!chassis_nh.getParam("normal_gyro_scale", normal_gyro_scale_))
    normal_gyro_scale_ = 0.15;
  if (!chassis_nh.getParam("low_gyro_scale", low_gyro_scale_))
    low_gyro_scale_ = 0.05;
  if (!chassis_nh.getParam("exchange_gyro_scale", exchange_gyro_scale_))
    exchange_gyro_scale_ = 0.12;
  // Drag
  ros::NodeHandle nh_drag(nh, "drag");
  drag_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_drag);
  // Joint7
  ros::NodeHandle nh_joint7(nh, "joint7");
  joint7_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_joint7);
  // Reversal
  ros::NodeHandle nh_reversal(nh, "reversal");
  reversal_command_sender_ = new rm_common::MultiDofCommandSender(nh_reversal);
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("calibration_gather", rpc_value);
  calibration_gather_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("pitch_calibration", rpc_value);
  pitch_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);

  gimbal_power_on_event_.setRising(boost::bind(&EngineerManual::gimbalOutputOn, this));
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
  shift_r_event_.setActiveHigh(boost::bind(&EngineerManual::shiftRPressing, this));
  shift_r_event_.setFalling(boost::bind(&EngineerManual::shiftRRelease, this));
  shift_event_.setActiveHigh(boost::bind(&EngineerManual::shiftPressing, this));
  shift_event_.setFalling(boost::bind(&EngineerManual::shiftRelease, this));
  mouse_left_event_.setFalling(boost::bind(&EngineerManual::mouseLeftRelease, this));
  mouse_right_event_.setFalling(boost::bind(&EngineerManual::mouseRightRelease, this));
}

void EngineerManual::updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  if (auto_exchange_->union_move_->getEnterFlag() != true && !auto_exchange_->getFinishFlag())
  {
    if (is_random_ore_)
    {
      servo_command_sender_->setLinearVel(dbus_data->ch_l_y, dbus_data->ch_l_x, dbus_data->wheel);
      servo_command_sender_->setAngularVel(dbus_data->ch_r_x, -dbus_data->ch_r_y, -angular_z_scale_);
    }
    else
    {
      servo_command_sender_->setLinearVel(dbus_data->wheel, -dbus_data->ch_l_x, dbus_data->ch_l_y);
      servo_command_sender_->setAngularVel(-angular_z_scale_, dbus_data->ch_r_y, dbus_data->ch_r_x);
    }
  }
  else
  {
    if (!auto_exchange_->union_move_->getFinishFlag())
    {
      std::vector<double> servo_scales;
      servo_scales.resize(3, 0.);
      servo_scales = auto_exchange_->union_move_->getServoScale();
      servo_command_sender_->setLinearVel(servo_scales[0], servo_scales[1], servo_scales[2]);
    }
    else
      servo_command_sender_->setZero();
  }
  ChassisGimbalManual::updatePc(dbus_data);
}

int EngineerManual::checkJointsLimit()
{
  //  joint1_.current_position = tf_buffer_.lookupTransform("base_link", "link1", ros::Time(0)).transform.translation.z;
  //  joint2_.current_position = tf_buffer_.lookupTransform("link1", "link2", ros::Time(0)).transform.tr  anslation.x;
  //  joint3_.current_position = tf_buffer_.lookupTransform("link2", "link3", ros::Time(0)).transform.translation.y;
  //  for (int i = 0; i < (int)joints_.size(); ++i)
  //  {
  //    if (joints_[i].judgeJointPosition())
  //      return (i + 1);
  //  }
  //  return 0;
  return 0;
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

void EngineerManual::run()
{
  ChassisGimbalManual::run();
  calibration_gather_->update(ros::Time::now());
  engineer_ui_pub_.publish(engineer_ui_);
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
  ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);

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
  if (servo_mode_ == SERVO)
    updateServo(data);
}

void EngineerManual::stoneNumCallback(const std_msgs::String ::ConstPtr& data)
{
  std::cout << stone_num_.size() << std::endl;
  if (data->data == "-1" && !stone_num_.empty())
    stone_num_.pop_back();
  else if (data->data == "WHITE")
    stone_num_.push_back("WHITE");
  else if (data->data == "YELLOW")
    stone_num_.push_back("YELLOW");
  else if (data->data == "MANUALLY")
    stone_num_.push_back("MANUALLY");
  if (stone_num_.size() >= 3)
    stone_num_.pop_back();
  engineer_ui_.stone_num = std::to_string(stone_num_.size());
}
void EngineerManual::gpioStateCallback(const rm_msgs::GpioData ::ConstPtr& data)
{
  gpio_state_.gpio_state = data->gpio_state;
  if (!gpio_state_.gpio_state[0])
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
  if (!reversal_motion_ && servo_mode_ == JOINT)
    reversal_command_sender_->setGroupValue(0., 0., 5 * dbus_data->ch_r_y, 5 * dbus_data->ch_l_x, 5 * dbus_data->ch_l_y,
                                            0.);
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
  {
    changeSpeedMode(EXCHANGE);
    servo_command_sender_->sendCommand(time);
    if (auto_exchange_->auto_servo_move_->getEnterFlag())
    {
      joint7_command_sender_->sendCommand(time);
    }
  }
  if (gimbal_mode_ == RATE)
    gimbal_cmd_sender_->sendCommand(time);
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

void EngineerManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  servo_mode_ = JOINT;
  gimbal_mode_ = RATE;
  gimbal_cmd_sender_->setZero();
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
  ROS_INFO_STREAM("START CALIBRATE");
}

void EngineerManual::leftSwitchDownFall()
{
  runStepQueue("EXCHANGE_WAIT");
  drag_command_sender_->off();
  drag_state_ = "off";
  engineer_ui_.drag_state = "off";
}

void EngineerManual::leftSwitchUpFall()
{
  runStepQueue("HOME_ZERO_STONE");
  drag_command_sender_->off();
  drag_state_ = "off";
  engineer_ui_.drag_state = "off";
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
  reversal_motion_ = false;
  change_flag_ = true;
  engineer_ui_.stone_num = std::to_string(stone_num_.size());
  if (prefix_ + root_ == "SMALL_ISLAND_TWO_ORE_L")
    changeSpeedMode(LOW);
  if ((prefix_ == "TAKE_WHEN_TWO_STONE" && root_ != "_AUTO_REVERSE") || prefix_ + root_ == "EXCHANGE_WAIT" ||
      (prefix_ == "TAKE_WHEN_ONE_STONE" && root_ != "_AUTO_REVERSE") || prefix_ + root_ == "BIG_ISLAND")
    enterServo();
  if (prefix_ + root_ == auto_exchange_->union_move_->getMotionName())
    auto_exchange_->union_move_->changeIsMotionFinish(true);
  if (prefix_ + root_ == "SMALL_ISLAND_TWO_ORE_L00")
    changeSpeedMode(NORMAL);
  ROS_INFO("%i", result->finish);

  operating_mode_ = MANUAL;
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
  prefix_ = "REVERSE";
  root_ = "_STONE";
  initMode();
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlXPress()
{
  if (drag_state_ == "on")
  {
    drag_command_sender_->off();
    drag_state_ = "off";
    engineer_ui_.drag_state = "off";
  }
  else
  {
    drag_command_sender_->on();
    drag_state_ = "on";
    engineer_ui_.drag_state = "on";
  }
  prefix_ = "DRAG";
  root_ = "_CAR";
  runStepQueue("DRAG_CAR");
}

void EngineerManual::ctrlEPress()
{
  prefix_ = "BIG";
  root_ = "_ISLAND";
  is_random_ore_ = true;
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlWPress()
{
  prefix_ = "CATCH_DROP";
  root_ = "";
  initMode();
  changeSpeedMode(EXCHANGE);
  runStepQueue(prefix_ + root_);
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

void EngineerManual::ctrlAPress()
{
  prefix_ = "SMALL_ISLAND";
  root_ = "_MID";
  initMode();
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlSPress()
{
  prefix_ = "SMALL_ISLAND";
  root_ = "_TWO_ORE_L";
  initMode();
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlDPress()
{
  prefix_ = "GROUND";
  root_ = "_STONE";
  initMode();
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlFPress()
{
  prefix_ = "EXCHANGE_";
  root_ = "WAIT";
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlGPress()
{
  switch (stone_num_.size())
  {
    case 0:
      root_ = "STORE_WHEN_ZERO_STONE";
      break;
    case 1:
      root_ = "STORE_WHEN_ONE_STONE";
      break;
  }
  prefix_ = "";
  initMode();
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
  ROS_INFO("STORE_STONE");
}

void EngineerManual::ctrlZPress()
{
}

void EngineerManual::ctrlZPressing()
{
  //  Use for auto exchange
  auto_exchange_->run();
  if (auto_exchange_->find_->getEnterFlag())
  {
    gimbal_mode_ = RATE;
    std::vector<double> gimbal_scale = auto_exchange_->find_->getGimbalScale();
    gimbal_cmd_sender_->setRate(gimbal_scale[0], gimbal_scale[1]);
  }
  if (auto_exchange_->pre_adjust_->getEnterFlag())
  {
    gimbal_cmd_sender_->setZero();
    geometry_msgs::Twist vel_msg = auto_exchange_->pre_adjust_->getChassisVelMsg();
    vel_cmd_sender_->setAngularZVel(vel_msg.angular.z);
    vel_cmd_sender_->setLinearXVel(vel_msg.linear.x);
    vel_cmd_sender_->setLinearYVel(vel_msg.linear.y);
  }
  if (auto_exchange_->union_move_->getEnterFlag())
  {
    vel_cmd_sender_->setLinearXVel(0.);
    vel_cmd_sender_->setLinearYVel(0.);
    vel_cmd_sender_->setAngularZVel(0.);
    if (!auto_exchange_->union_move_->getIsMotionStart())
    {
      auto_exchange_->union_move_->changeIsMotionStart(true);
      runStepQueue(auto_exchange_->union_move_->getMotionName());
    }
  }
}

void EngineerManual::ctrlZRelease()
{
  auto_exchange_->init();
  gimbal_cmd_sender_->setZero();
  chassis_cmd_sender_->setZero();

  gimbal_mode_ = DIRECT;
  runStepQueue("GIMBAL_ISLAND");
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  enterServo();
}

void EngineerManual::ctrlCPress()
{
  runStepQueue("DELETE_SCENE");
  action_client_.cancelAllGoals();
}

void EngineerManual::shiftVPress()
{
  if (gripper_state_ == "open")
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
}

void EngineerManual::ctrlBPress()
{
  initMode();
  switch (stone_num_.size())
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
  }
  ROS_INFO("RUN_HOME");
  drag_command_sender_->off();
  drag_state_ = "off";
  engineer_ui_.drag_state = "off";
  prefix_ = "";
  runStepQueue(prefix_ + root_);
}

void EngineerManual::qPressing()
{
  vel_cmd_sender_->setAngularZVel(gyro_scale_);
}

void EngineerManual::qRelease()
{
  vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::ePressing()
{
  vel_cmd_sender_->setAngularZVel(-gyro_scale_);
}

void EngineerManual::eRelease()
{
  vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::zPressing()
{
  angular_z_scale_ = 0.3;
}

void EngineerManual::zRelease()
{
  angular_z_scale_ = 0.;
}

void EngineerManual::cPressing()
{
  angular_z_scale_ = -0.3;
}

void EngineerManual::cRelease()
{
  angular_z_scale_ = 0.;
}

void EngineerManual::rPress()
{
  if (stone_num_.size() != 2)
    stone_num_.push_back("MANUALLY");
  else
    stone_num_.clear();
  engineer_ui_.stone_num = std::to_string(stone_num_.size());
  ROS_INFO("Stone num is %d", static_cast<int>(stone_num_.size()));
}

void EngineerManual::xPress()
{
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
  runStepQueue("PITCH_REVERSAL_PI_2");
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
  reversal_command_sender_->setGroupValue(0., 0., -3., 0., 0., 0.);
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
  changeSpeedMode(FAST);
}
void EngineerManual::shiftRelease()
{
  changeSpeedMode(NORMAL);
}
void EngineerManual::shiftFPress()
{
  runStepQueue("GIMBAL_SMALL_SCREEN");
  ROS_INFO("enter gimbal EXCHANGE_GIMBAL");
}

void EngineerManual::shiftCPress()
{
  if (servo_mode_ == SERVO)
  {
    initMode();
  }
  else
  {
    enterServo();
  }
}

void EngineerManual::initMode()
{
  servo_mode_ = JOINT;
  gimbal_mode_ = DIRECT;
  is_random_ore_ = false;
  changeSpeedMode(NORMAL);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
}

void EngineerManual::enterServo()
{
  servo_mode_ = SERVO;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(EXCHANGE);
  servo_reset_caller_->callService();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
  chassis_cmd_sender_->getMsg()->command_source_frame = "link5";
}

void EngineerManual::shiftZPress()
{
  runStepQueue("GIMBAL_ISLAND");
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  ROS_INFO("enter gimbal REVERSAL_GIMBAL");
}
void EngineerManual::ctrlVPress()
{
  prefix_ = "EXCHANGE_SPHERE";
  root_ = "";
  runStepQueue(prefix_ + root_);
}

void EngineerManual::ctrlVRelease()
{
}

void EngineerManual::shiftBPress()
{
  runStepQueue("GIMBAL_SIDE");
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  ROS_INFO("enter gimbal BACK_GIMBAL");
}

void EngineerManual::shiftBRelease()
{
}

void EngineerManual::shiftRRelease()
{
  //  servo_move_info_.quitExchange();
}

void EngineerManual::shiftRPressing()
{
  //  enterServoAutoMove();
}

void EngineerManual::shiftXPress()
{
  runStepQueue("GIMBAL_GROUND");
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  ROS_INFO("enter gimbal GROUND_GIMBAL");
}

void EngineerManual::shiftGPress()
{
  if (stone_num_.empty())
  {
    root_ = "NO STONE!!";
  }
  else
  {
    if (stone_num_.size() == 1)
      prefix_ = "TAKE_WHEN_ONE_STONE";
    else if (stone_num_.size() == 2)
      prefix_ = "TAKE_WHEN_TWO_STONE";
    if (stone_num_.back() == "YELLOW")
      root_ = "_NO_REVERSE";
    else if (stone_num_.back() == "WHITE")
      root_ = "_AUTO_REVERSE";
    else if (stone_num_.back() == "MANUALLY")
      root_ = "_NO_REVERSE";
  }
  engineer_ui_.stone_num = std::to_string(stone_num_.size());
  changeSpeedMode(LOW);
  runStepQueue(prefix_ + root_);
}
}  // namespace rm_manual
