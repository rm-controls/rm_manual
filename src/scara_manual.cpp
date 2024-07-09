//
// Created by cch on 24-5-31.
//
#include "rm_manual/scara_manual.h"

namespace rm_manual
{
ScaraManual::ScaraManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalManual(nh, nh_referee)
  , operating_mode_(MANUAL)
  , action_client_("/engineer_middleware/move_steps", true)
{
  engineer_ui_pub_ = nh.advertise<rm_msgs::EngineerUi>("/engineer_ui", 10);
  ROS_INFO("Waiting for middleware to start.");
  action_client_.waitForServer();
  ROS_INFO("Middleware started.");
  stone_num_sub_ = nh.subscribe<std_msgs::String>("/stone_num", 10, &ScaraManual::stoneNumCallback, this);
  gripper_state_sub_ = nh.subscribe<rm_msgs::GpioData>("/controllers/gpio_controller/gpio_states", 10,
                                                       &ScaraManual::gpioStateCallback, this);

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
  // Calibration
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("calibration_gather", rpc_value);
  calibration_gather_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  left_switch_up_event_.setFalling(boost::bind(&ScaraManual::leftSwitchUpFall, this));
  left_switch_up_event_.setRising(boost::bind(&ScaraManual::leftSwitchUpRise, this));
  left_switch_down_event_.setFalling(boost::bind(&ScaraManual::leftSwitchDownFall, this));
  left_switch_down_event_.setRising(boost::bind(&ScaraManual::leftSwitchDownRise, this));
  ctrl_a_event_.setRising(boost::bind(&ScaraManual::ctrlAPress, this));
  ctrl_b_event_.setRising(boost::bind(&ScaraManual::ctrlBPress, this));
  ctrl_c_event_.setRising(boost::bind(&ScaraManual::ctrlCPress, this));
  ctrl_b_event_.setActiveHigh(boost::bind(&ScaraManual::ctrlBPressing, this));
  ctrl_b_event_.setFalling(boost::bind(&ScaraManual::ctrlBRelease, this));
  ctrl_d_event_.setRising(boost::bind(&ScaraManual::ctrlDPress, this));
  ctrl_e_event_.setRising(boost::bind(&ScaraManual::ctrlEPress, this));
  ctrl_f_event_.setRising(boost::bind(&ScaraManual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&ScaraManual::ctrlGPress, this));
  ctrl_q_event_.setRising(boost::bind(&ScaraManual::ctrlQPress, this));
  ctrl_r_event_.setRising(boost::bind(&ScaraManual::ctrlRPress, this));
  ctrl_s_event_.setRising(boost::bind(&ScaraManual::ctrlSPress, this));
  ctrl_v_event_.setRising(boost::bind(&ScaraManual::ctrlVPress, this));
  ctrl_v_event_.setFalling(boost::bind(&ScaraManual::ctrlVRelease, this));
  ctrl_w_event_.setRising(boost::bind(&ScaraManual::ctrlWPress, this));
  ctrl_x_event_.setRising(boost::bind(&ScaraManual::ctrlXPress, this));
  ctrl_z_event_.setRising(boost::bind(&ScaraManual::ctrlZPress, this));
  b_event_.setActiveHigh(boost::bind(&ScaraManual::bPressing, this));
  b_event_.setFalling(boost::bind(&ScaraManual::bRelease, this));
  c_event_.setActiveHigh(boost::bind(&ScaraManual::cPressing, this));
  c_event_.setFalling(boost::bind(&ScaraManual::cRelease, this));
  e_event_.setActiveHigh(boost::bind(&ScaraManual::ePressing, this));
  e_event_.setFalling(boost::bind(&ScaraManual::eRelease, this));
  f_event_.setRising(boost::bind(&ScaraManual::fPress, this));
  f_event_.setFalling(boost::bind(&ScaraManual::fRelease, this));
  g_event_.setRising(boost::bind(&ScaraManual::gPress, this));
  g_event_.setFalling(boost::bind(&ScaraManual::gRelease, this));
  q_event_.setActiveHigh(boost::bind(&ScaraManual::qPressing, this));
  q_event_.setFalling(boost::bind(&ScaraManual::qRelease, this));
  r_event_.setRising(boost::bind(&ScaraManual::rPress, this));
  v_event_.setActiveHigh(boost::bind(&ScaraManual::vPressing, this));
  v_event_.setFalling(boost::bind(&ScaraManual::vRelease, this));
  x_event_.setRising(boost::bind(&ScaraManual::xPress, this));
  z_event_.setActiveHigh(boost::bind(&ScaraManual::zPressing, this));
  z_event_.setFalling(boost::bind(&ScaraManual::zRelease, this));
  shift_event_.setActiveHigh(boost::bind(&ScaraManual::shiftPressing, this));
  shift_event_.setFalling(boost::bind(&ScaraManual::shiftRelease, this));
  shift_b_event_.setRising(boost::bind(&ScaraManual::shiftBPress, this));
  shift_b_event_.setFalling(boost::bind(&ScaraManual::shiftBRelease, this));
  shift_c_event_.setRising(boost::bind(&ScaraManual::shiftCPress, this));
  shift_e_event_.setRising(boost::bind(&ScaraManual::shiftEPress, this));
  shift_f_event_.setRising(boost::bind(&ScaraManual::shiftFPress, this));
  shift_g_event_.setRising(boost::bind(&ScaraManual::shiftGPress, this));
  shift_q_event_.setRising(boost::bind(&ScaraManual::shiftQPress, this));
  shift_r_event_.setRising(boost::bind(&ScaraManual::shiftRPress, this));
  shift_r_event_.setFalling(boost::bind(&ScaraManual::shiftRRelease, this));
  shift_v_event_.setRising(boost::bind(&ScaraManual::shiftVPress, this));
  shift_v_event_.setFalling(boost::bind(&ScaraManual::shiftVRelease, this));
  shift_x_event_.setRising(boost::bind(&ScaraManual::shiftXPress, this));
  shift_z_event_.setRising(boost::bind(&ScaraManual::shiftZPress, this));
  shift_z_event_.setFalling(boost::bind(&ScaraManual::shiftZRelease, this));

  mouse_left_event_.setFalling(boost::bind(&ScaraManual::mouseLeftRelease, this));
  mouse_right_event_.setFalling(boost::bind(&ScaraManual::mouseRightRelease, this));
}

void ScaraManual::run()
{
  ChassisGimbalManual::run();
  calibration_gather_->update(ros::Time::now());
  if (engineer_ui_ != old_ui_)
  {
    engineer_ui_pub_.publish(engineer_ui_);
    old_ui_ = engineer_ui_;
  }
}

void ScaraManual::changeSpeedMode(SpeedMode speed_mode)
{
  switch (speed_mode)
  {
    case LOW:
      speed_change_scale_ = low_speed_scale_;
      gyro_scale_ = low_gyro_scale_;
      break;
    case NORMAL:
      speed_change_scale_ = normal_speed_scale_;
      gyro_scale_ = normal_gyro_scale_;
      break;
    case FAST:
      speed_change_scale_ = fast_speed_scale_;
      gyro_scale_ = fast_gyro_scale_;
      break;
    case EXCHANGE:
      speed_change_scale_ = exchange_speed_scale_;
      gyro_scale_ = exchange_gyro_scale_;
      break;
    default:
      speed_change_scale_ = normal_speed_scale_;
      gyro_scale_ = normal_gyro_scale_;
      break;
  }
}

void ScaraManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
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

void ScaraManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
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

void ScaraManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  checkKeyboard(dbus_data);
  left_switch_up_event_.update(dbus_data->s_l == rm_msgs::DbusData::UP);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  if (servo_mode_ == JOINT)
    vel_cmd_sender_->setAngularZVel(-dbus_data->m_x * gimbal_scale_);
}

void ScaraManual::updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  servo_command_sender_->setLinearVel(-dbus_data->wheel, -dbus_data->ch_l_x, -dbus_data->ch_l_y);
  servo_command_sender_->setAngularVel(-angular_z_scale_, dbus_data->ch_r_y, -dbus_data->ch_r_x);
}

void ScaraManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
  if (servo_mode_ == SERVO)
    updateServo(data);
}

void ScaraManual::stoneNumCallback(const std_msgs::String::ConstPtr& data)
{
}

void ScaraManual::gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data)
{
  gpio_state_.gpio_state = data->gpio_state;
  if (gpio_state_.gpio_state[0])
  {
    gripper_state_ = "open";
    engineer_ui_.gripper_state = "OPEN";
  }
  else
  {
    gripper_state_ = "close";
    engineer_ui_.gripper_state = "CLOSED";
  }
}

void ScaraManual::sendCommand(const ros::Time& time)
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

void ScaraManual::runStepQueue(const std::string& step_queue_name)
{
  rm_msgs::EngineerGoal goal;
  goal.step_queue_name = step_queue_name;
  if (action_client_.isServerConnected())
  {
    if (operating_mode_ == MANUAL)
      action_client_.sendGoal(goal, boost::bind(&ScaraManual::actionDoneCallback, this, _1, _2),
                              boost::bind(&ScaraManual::actionActiveCallback, this),
                              boost::bind(&ScaraManual::actionFeedbackCallback, this, _1));
    operating_mode_ = MIDDLEWARE;
  }
  else
    ROS_ERROR("Can not connect to middleware");
}

void ScaraManual::actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback)
{
}

void ScaraManual::actionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                     const rm_msgs::EngineerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %i", result->finish);
  ROS_INFO("Done %s", (prefix_ + root_).c_str());
  change_flag_ = true;
  ROS_INFO("%i", result->finish);
  operating_mode_ = MANUAL;
  if (root_ == "HOME")
  {
    initMode();
    changeSpeedMode(NORMAL);
  }
}

void ScaraManual::enterServo()
{
  servo_mode_ = SERVO;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(EXCHANGE);
  servo_reset_caller_->callService();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  action_client_.cancelAllGoals();
  chassis_cmd_sender_->getMsg()->command_source_frame = "link4";
  engineer_ui_.control_mode = "SERVO";
}

void ScaraManual::initMode()
{
  servo_mode_ = JOINT;
  gimbal_mode_ = DIRECT;
  changeSpeedMode(NORMAL);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  engineer_ui_.control_mode = "NORMAL";
}

void ScaraManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  action_client_.cancelAllGoals();
}

void ScaraManual::gimbalOutputOn()
{
  ChassisGimbalManual::gimbalOutputOn();
}

void ScaraManual::chassisOutputOn()
{
  if (operating_mode_ == MIDDLEWARE)
    action_client_.cancelAllGoals();
}

//-------------------controller input-------------------
void ScaraManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  gimbal_mode_ = DIRECT;
  servo_mode_ = JOINT;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
void ScaraManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  servo_mode_ = JOINT;
  gimbal_mode_ = RATE;
  gimbal_cmd_sender_->setZero();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}
void ScaraManual::rightSwitchDownRise()
{
  ChassisGimbalManual::rightSwitchDownRise();
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  servo_mode_ = SERVO;
  gimbal_mode_ = RATE;
  servo_reset_caller_->callService();
  action_client_.cancelAllGoals();
  ROS_INFO_STREAM("servo_mode");
}

void ScaraManual::leftSwitchUpRise()
{
  prefix_ = "";
  root_ = "CALIBRATION";
  calibration_gather_->reset();
  engineer_ui_.stone_num = 0;
  engineer_ui_.gripper_state = "CLOSED";
  engineer_ui_.control_mode = "NORMAL";
  ROS_INFO_STREAM("START CALIBRATE");
}
void ScaraManual::leftSwitchUpFall()
{
  runStepQueue("HOME");
  runStepQueue("CLOSE_GRIPPER");
}

void ScaraManual::leftSwitchDownRise()
{
  if (gripper_state_ == "close")
  {
    runStepQueue("OPEN_GRIPPER");
    engineer_ui_.gripper_state = "OPEN";
  }
  else
  {
    runStepQueue("CLOSE_GRIPPER");
    engineer_ui_.gripper_state = "CLOSED";
  }
}
void ScaraManual::leftSwitchDownFall()
{
}

//--------------------- keyboard input ------------------------
// mouse input
void ScaraManual::mouseLeftRelease()
{
  if (change_flag_)
  {
    root_ += "0";
    change_flag_ = false;
    runStepQueue(prefix_ + root_);
    ROS_INFO("Finished %s", (prefix_ + root_).c_str());
  }
}

void ScaraManual::mouseRightRelease()
{
  runStepQueue(prefix_ + root_);
  ROS_INFO("Finished %s", (prefix_ + root_).c_str());
}

void ScaraManual::bPressing()
{
}

void ScaraManual::bRelease()
{
}
void ScaraManual::cPressing()
{
}
void ScaraManual::cRelease()
{
}
void ScaraManual::ePressing()
{
}
void ScaraManual::eRelease()
{
}
void ScaraManual::fPress()
{
}
void ScaraManual::fRelease()
{
}
void ScaraManual::gPress()
{
}
void ScaraManual::gRelease()
{
}
void ScaraManual::qPressing()
{
}
void ScaraManual::qRelease()
{
}
void ScaraManual::rPress()
{
}
void ScaraManual::vPressing()
{
}
void ScaraManual::vRelease()
{
}
void ScaraManual::xPress()
{
}
void ScaraManual::zPressing()
{
}
void ScaraManual::zRelease()
{
}

//---------------------  CTRL  ---------------------
void ScaraManual::ctrlAPress()
{
}
void ScaraManual::ctrlBPress()
{
}
void ScaraManual::ctrlBPressing()
{
}
void ScaraManual::ctrlBRelease()
{
}
void ScaraManual::ctrlCPress()
{
}
void ScaraManual::ctrlDPress()
{
}
void ScaraManual::ctrlEPress()
{
}
void ScaraManual::ctrlFPress()
{
}
void ScaraManual::ctrlGPress()
{
}
void ScaraManual::ctrlQPress()
{
}
void ScaraManual::ctrlRPress()
{
}
void ScaraManual::ctrlSPress()
{
}
void ScaraManual::ctrlVPress()
{
}
void ScaraManual::ctrlVRelease()
{
}
void ScaraManual::ctrlWPress()
{
}
void ScaraManual::ctrlXPress()
{
}
void ScaraManual::ctrlZPress()
{
}

//---------------  SHIFT  --------------------------

void ScaraManual::shiftPressing()
{
}
void ScaraManual::shiftRelease()
{
}
void ScaraManual::shiftBPress()
{
}
void ScaraManual::shiftBRelease()
{
}
void ScaraManual::shiftCPress()
{
}
void ScaraManual::shiftEPress()
{
}
void ScaraManual::shiftFPress()
{
}
void ScaraManual::shiftGPress()
{
}
void ScaraManual::shiftQPress()
{
}
void ScaraManual::shiftRPress()
{
}
void ScaraManual::shiftRRelease()
{
}
void ScaraManual::shiftVPress()
{
}
void ScaraManual::shiftVRelease()
{
}
void ScaraManual::shiftXPress()
{
}
void ScaraManual::shiftZPress()
{
}
void ScaraManual::shiftZRelease()
{
}

}  // namespace rm_manual
