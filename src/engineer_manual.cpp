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
  // Command sender
  ros::NodeHandle nh_drag(nh, "drag");
  drag_command_sender_ = new rm_common::JointPositionBinaryCommandSender(nh_drag);
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
  ctrl_f_event_.setRising(boost::bind(&EngineerManual::ctrlFPress, this));
  ctrl_g_event_.setRising(boost::bind(&EngineerManual::ctrlGPress, this));
  z_event_.setRising(boost::bind(&EngineerManual::zPress, this));
  x_event_.setRising(boost::bind(&EngineerManual::xPress, this));
  c_event_.setRising(boost::bind(&EngineerManual::cPress, this));
  v_event_.setRising(boost::bind(&EngineerManual::vPress, this));
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
  mouse_left_event_.setFalling(boost::bind(&EngineerManual::mouseLeftRelease, this));
  mouse_right_event_.setFalling(boost::bind(&EngineerManual::mouseRightRelease, this));
}

void EngineerManual::run()
{
  ChassisGimbalManual::run();
  power_on_calibration_->update(ros::Time::now(), state_ != PASSIVE);
  arm_calibration_->update(ros::Time::now());
  updateServo();
}

void EngineerManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalManual::checkKeyboard(dbus_data);
  ctrl_q_event_.update(dbus_data_.key_ctrl & dbus_data_.key_q);
  ctrl_a_event_.update(dbus_data_.key_ctrl & dbus_data_.key_a);
  ctrl_z_event_.update(dbus_data_.key_ctrl & dbus_data_.key_z);
  ctrl_w_event_.update(dbus_data_.key_ctrl & dbus_data_.key_w);
  ctrl_s_event_.update(dbus_data_.key_ctrl & dbus_data_.key_s);
  ctrl_x_event_.update(dbus_data_.key_ctrl & dbus_data_.key_x);
  ctrl_e_event_.update(dbus_data_.key_ctrl & dbus_data_.key_e);
  ctrl_d_event_.update(dbus_data_.key_ctrl & dbus_data_.key_d);
  ctrl_c_event_.update(dbus_data_.key_ctrl & dbus_data_.key_c);
  ctrl_b_event_.update(dbus_data_.key_ctrl & dbus_data_.key_b);
  ctrl_r_event_.update(dbus_data_.key_ctrl & dbus_data_.key_r);
  ctrl_g_event_.update(dbus_data_.key_g & dbus_data_.key_ctrl);
  ctrl_f_event_.update(dbus_data_.key_f & dbus_data_.key_ctrl);

  z_event_.update(dbus_data_.key_z & !dbus_data_.key_ctrl & !dbus_data_.key_shift);
  x_event_.update(dbus_data_.key_x & !dbus_data_.key_ctrl & !dbus_data_.key_shift);
  c_event_.update(dbus_data_.key_c & !dbus_data_.key_ctrl & !dbus_data_.key_shift);
  v_event_.update(dbus_data_.key_v & !dbus_data_.key_ctrl & !dbus_data_.key_shift);
  b_event_.update(dbus_data_.key_b & !dbus_data_.key_ctrl & !dbus_data_.key_shift);
  g_event_.update(dbus_data_.key_g & !dbus_data_.key_ctrl & !dbus_data_.key_shift);
  f_event_.update(dbus_data_.key_f & !dbus_data_.key_ctrl & !dbus_data_.key_shift);
  r_event_.update(dbus_data_.key_r & !dbus_data_.key_ctrl & !dbus_data_.key_shift);

  shift_z_event_.update(dbus_data_.key_shift & dbus_data_.key_z);
  shift_x_event_.update(dbus_data_.key_shift & dbus_data_.key_x);
  shift_c_event_.update(dbus_data_.key_shift & dbus_data_.key_c);
  shift_v_event_.update(dbus_data_.key_shift & dbus_data_.key_v);
  shift_b_event_.update(dbus_data_.key_shift & dbus_data_.key_b);
  shift_q_event_.update(dbus_data_.key_shift & dbus_data_.key_q);
  shift_e_event_.update(dbus_data_.key_shift & dbus_data_.key_e);
  shift_r_event_.update(dbus_data_.key_shift & dbus_data_.key_r);
  shift_event_.update(dbus_data_.key_shift & !dbus_data_.key_ctrl);

  mouse_left_event_.update(dbus_data_.p_l);
  mouse_right_event_.update(dbus_data_.p_r);
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
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  vel_cmd_sender_->setAngularZVel(-dbus_data_.m_x);
}

void EngineerManual::sendCommand(const ros::Time& time)
{
  if (operating_mode_ == MANUAL)
  {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    if (servo_mode_ == SERVO)
    //      servo_command_sender_->sendCommand(time);
    {
      ;
    }
  }
  if (gimbal_mode_ == RATE)
  {
    gimbal_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->setZero();
    vel_cmd_sender_->sendCommand(time);
  }
}

void EngineerManual::updateServo()
{
  servo_command_sender_->setLinearVel(dbus_data_.ch_l_y, -dbus_data_.ch_l_x, -dbus_data_.wheel);
  servo_command_sender_->setAngularVel(-dbus_data_.ch_r_x, -dbus_data_.ch_r_y, angular_z_scale_);
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
  toward_change_mode_ = 0;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::rightSwitchUpRise()
{
  ChassisGimbalManual::rightSwitchUpRise();
  gimbal_mode_ = DIRECT;
  toward_change_mode_ = 0;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void EngineerManual::leftSwitchUpRise()
{
}

void EngineerManual::leftSwitchUpFall()
{
  runStepQueue("NORMAL_HOME0");
}

void EngineerManual::leftSwitchDownFall()
{
  arm_calibration_->reset();
  power_on_calibration_->reset();
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
        prefix_ = "NORMAL_";
      if (prefix_num_ == 2)
        prefix_ = "SKY_";
      if (prefix_num_ == 3)
        prefix_ = "NO!!";
      if (prefix_num_ == 4)
        prefix_ = "NO!!";
      break;
    case (3):
      if (prefix_num_ == 1)
        prefix_ = "NORMAL_";
      if (prefix_num_ == 2)
        prefix_ = "STORED_";
      if (prefix_num_ == 3)
        prefix_ = "EXCHANGE_";
      if (prefix_num_ == 4)
        prefix_ = "NO!!";
      break;
    case (4):
      if (prefix_num_ == 1)
        prefix_ = "LONG_";
      if (prefix_num_ == 2)
        prefix_ = "SHORT_";
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
      if (root_num_ == 4)
        prefix_ = "WAIT_";
      break;
    case (1):
      if (root_num_ == 1)
        prefix_ = "LF_";
      if (root_num_ == 2)
        prefix_ = "NORMAL_";
      if (root_num_ == 3)
        prefix_ = "NORMAL_";
      if (root_num_ == 4)
        prefix_ = "LONG_";
      break;
    case (2):
      if (root_num_ == 1)
        prefix_ = "MID_";
      if (root_num_ == 2)
        prefix_ = "SKY_";
      if (root_num_ == 3)
        prefix_ = "STORED_";
      if (root_num_ == 4)
        prefix_ = "SHORT_";
      break;
    case (3):
      if (root_num_ == 1)
        prefix_ = "RT_";
      if (root_num_ == 2)
        prefix_ = "NO!!";
      if (root_num_ == 3)
        prefix_ = "EXCHANGE_";
      if (root_num_ == 4)
        prefix_ = "NO!!";
      break;
    case (4):
      if (root_num_ == 1)
        prefix_ = "READY_";
      if (root_num_ == 2)
        prefix_ = "NO!!";
      if (root_num_ == 3)
        prefix_ = "NO!!";
      if (root_num_ == 4)
        prefix_ = "NO!!";
      break;
  }
}
void EngineerManual::mouseLeftRelease()
{
  root_ += "0";
  runStepQueue(prefix_ + root_);
  ROS_INFO("Finished %s", (prefix_ + root_).c_str());
}

void EngineerManual::mouseRightRelease()
{
  runStepQueue(prefix_ + root_);
  ROS_INFO("Finished %s", (prefix_ + root_).c_str());
}
void EngineerManual::ctrlQPress()
{
  prefix_num_ = 1;
  judgePrefix();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlWPress()
{
  prefix_num_ = 2;
  judgePrefix();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlEPress()
{
  prefix_num_ = 3;
  judgePrefix();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlRPress()
{
  prefix_num_ = 4;
  judgePrefix();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlAPress()
{
  root_num_ = 1;
  root_ = "SKY_ISLAND";
  judgeRoot();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlSPress()
{
  root_num_ = 1;
  root_ = "BIG_ISLAND";
  judgeRoot();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlDPress()
{
  prefix_ = "";
  root_ = "EXCHANGE";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlFPress()
{
  root_num_ = 1;
  root_ = "GROUND_STONE";
  judgeRoot();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlGPress()
{
  root_num_ = 4;
  root_ = "GAIN_BARRIER";
  judgeRoot();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlZPress()
{
  prefix_ = "";
  root_ = "STORE_STONE";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlXPress()
{
  root_num_ = 2;
  root_ = "GAIN_STORE_STONE";
  judgeRoot();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlCPress()
{
  action_client_.cancelAllGoals();
  runStepQueue("DELETE_SCENE");
  ROS_INFO("DELETE_SCENE and CANCEL");
}

void EngineerManual::ctrlBPress()
{
  root_num_ = 3;
  root_ = "HOME";
  judgeRoot();
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::zPress()
{
  if (card_command_sender_->getState())
  {
    card_command_sender_->off();
    ROS_INFO("long_card off");
  }
  else
  {
    card_command_sender_->long_on();
    ROS_INFO("long_card on");
  }
}

void EngineerManual::xPress()
{
  if (card_command_sender_->getState())
  {
    card_command_sender_->off();
    ROS_INFO("short_card off");
  }
  else
  {
    card_command_sender_->short_on();
    ROS_INFO("short_card on");
  }
}

void EngineerManual::cPress()
{
  if (drag_command_sender_->getState())
  {
    drag_command_sender_->off();
    ROS_INFO("drag off");
  }
  else
  {
    drag_command_sender_->on();
    ROS_INFO("drag on");
  }
}

void EngineerManual::rPress()
{
  arm_calibration_->reset();
  power_on_calibration_->reset();
  ROS_INFO("Calibrated");
}

void EngineerManual::vPress()
{
  {
    servo_mode_ = SERVO;
    servo_reset_caller_->callService();
    ROS_INFO("ENTER SERVO");
  }
}

void EngineerManual::gPress()
{
  runStepQueue("CLOSE_GRIPPER");
  ROS_INFO("close gripper");
}
void EngineerManual::gRelease()
{
  runStepQueue("OPEN_GRIPPER");
  ROS_INFO("open gripper");
}
void EngineerManual::fPress()
{
  // enter gimbal rate
  gimbal_mode_ = RATE;
  ROS_INFO("Enter gimbal rate");
}
void EngineerManual::fRelease()
{
  // exit gimbal rate
  gimbal_mode_ = DIRECT;
  ROS_INFO("exit gimbal rate");
}
void EngineerManual::shiftPressing()
{
  speed_change_mode_ = 1;
}
void EngineerManual::shiftRelease()
{
  speed_change_mode_ = 0;
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
void EngineerManual::shiftZPress()
{
  toward_change_mode_ = 0;
  runStepQueue("WALK_GIMBAL");
  ROS_INFO("enter gimbal WALK_GIMBAL");
}
void EngineerManual::shiftXPress()
{
  toward_change_mode_ = 0;
  runStepQueue("BIG_STONE_GIMBAL");
  ROS_INFO("enter gimbal BIG_STONE_GIMBAL");
}
void EngineerManual::shiftCPress()
{
  toward_change_mode_ = 1;
  runStepQueue("BACK_GIMBAL");
  ROS_INFO("enter gimbal BACK_GIMBAL");
}
void EngineerManual::shiftVPress()
{
  toward_change_mode_ = 1;
  runStepQueue("SKY_GIMBAL");
  ROS_INFO("enter gimbal SKY_GIMBAL");
}

}  // namespace rm_manual
