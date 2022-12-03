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
  ros::NodeHandle nh_reversal(nh,"reversal");
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
  ctrl_v_event_.setRising(boost::bind(&EngineerManual::ctrlVPress,this));
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
  ctrl_r_event_.setRising(boost::bind(&EngineerManual::ctrlRPress, this));
  shift_q_event_.setRising(boost::bind(&EngineerManual::shiftQPress, this));
  shift_e_event_.setRising(boost::bind(&EngineerManual::shiftEPress, this));
  shift_z_event_.setRising(boost::bind(&EngineerManual::shiftZPress, this));
  shift_c_event_.setRising(boost::bind(&EngineerManual::shiftCPress, this));
  shift_v_event_.setRising(boost::bind(&EngineerManual::shiftVPress, this));
  shift_v_event_.setFalling(boost::bind(&EngineerManual::shiftVRelease, this));
  shift_b_event_.setRising(boost::bind(&EngineerManual::shiftBPress, this));
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
  ctrl_e_event_.update(dbus_data_.key_ctrl & dbus_data_.key_e);
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
  }
  if (servo_mode_ == SERVO)
    servo_command_sender_->sendCommand(time);
  if (gimbal_mode_ == RATE)
  {
    gimbal_cmd_sender_->sendCommand(time);
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
  gimbal_mode_ = DIRECT;
  servo_reset_caller_->callService();
  action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchMidRise()
{
  ChassisGimbalManual::rightSwitchMidRise();
  servo_mode_ = JOINT;
  gimbal_mode_ = RATE;
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

void EngineerManual::leftSwitchDownFall()
{
  switch (stone_num_) {
      case 0: root_ = "HOME0";
      case 1: root_ = "HOME1";
      case 2: root_ = "HOME2";
  }
  runStepQueue(root_);
  ROS_INFO("RUN_HOME");
}

void EngineerManual::leftSwitchUpFall()
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
  arm_calibration_->reset();
  power_on_calibration_->reset();
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
  root_ = "GROUND_STONE";
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlFPress()
{
  root_ = "EXCHANGE";
  stone_num_ -= 1;
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlGPress()
{
  switch (stone_num_) {
      case 0: root_ = "STORE_STONE0";
      case 1: root_ = "STORE_STONE1";
      case 2: root_ = "STORE_STONE2";
  }
  prefix_ = "";
  stone_num_ += 1;
  ROS_INFO("STORE_STONE");
}

void EngineerManual::ctrlZPress()
{
  //reversal
  toward_change_mode_ = 0;
  runStepQueue("REVERSAL_GIMBAL");
  ROS_INFO("enter gimbal WALK_GIMBAL");
  ROS_INFO("AUTO_REVERSAL");
}

void EngineerManual::ctrlXPress()
{
  //drag
  if (drag_state_ == 0)
  {
    drag_command_sender_->on();
    ROS_INFO("DRAG UP");
    drag_state_ = 1;
  }
  else {
      drag_command_sender_->off();
      ROS_INFO("DRAG DOWN");
      drag_state_ = 0;
  }
}

void EngineerManual::ctrlCPress()
{
  //servo
  if (servo_mode_ == JOINT)
  {
      servo_mode_ = SERVO;
      servo_reset_caller_->callService();
      ROS_INFO("ENTER SERVO");
  } else{
      servo_mode_ = JOINT;
      ROS_INFO("OUT SERVO");
  }
}

void EngineerManual::ctrlVPress()
{
    //gripper
    if (gripper_state_ == 0)
    {
        runStepQueue("OPEN_GRIPPER");
        ROS_INFO("GRIPPER OPEN");
        gripper_state_ = 1;
    }
    else
    {
        runStepQueue("CLOSE_GRIPPER");
        ROS_INFO("GRIPPER CLOSE");
        gripper_state_ = 0;
    }
}
void EngineerManual::ctrlBPress()
{
  switch (stone_num_) {
      case 0: root_ = "HOME0";
      case 1: root_ = "HOME1";
      case 2: root_ = "HOME2";
  }
  ROS_INFO("RUN_HOME");
  runStepQueue(root_);
}

void EngineerManual::zPress()
{
  angular_z_scale_ = 0.5;
}

void EngineerManual::xPress()
{
  angular_z_scale_ = -0.5;
}

void EngineerManual::cPress()
{
  action_client_.cancelAllGoals();
  gimbal_mode_ = DIRECT;
  ROS_INFO("CANCEL ALL GOAL");
}

void EngineerManual::rPress()
{
}

void EngineerManual::vPress()
{
   //reversal roll
   reversal_command_sender_->setGroupVel(1.0,0.,0.);
   ROS_INFO("REVERSAL ROLL");
}

void EngineerManual::gPress()
{
  //reversal pitch
  reversal_command_sender_->setGroupVel(0.,1.0,0.);
  ROS_INFO("REVERSAL ROLL");
}
void EngineerManual::gRelease()
{
  //reversal down
  reversal_command_sender_->setGroupVel(0.,0.,-1.0);
  ROS_INFO("REVERSAL IN");
}
void EngineerManual::fPress()
{
  //reversal up
  reversal_command_sender_->setGroupVel(0.,0.,1.0);
  ROS_INFO("REVERSAL IN");
}
void EngineerManual::shiftPressing()
{
  speed_change_mode_ = 1;
  speed_change_scale_ = 0.1;
}
void EngineerManual::shiftRelease()
{
    speed_change_mode_ = 0;
}
void EngineerManual::shiftQPress()
{
    toward_change_mode_ = 0;
    runStepQueue("ISLAND_GIMBAL");
    ROS_INFO("enter gimbal ISLAND_GIMBAL");
}
void EngineerManual::shiftEPress()
{
    toward_change_mode_ = 0;
    runStepQueue(" SKY_GIMBAL");
    ROS_INFO("enter gimbal SKY_GIMBAL");
}

void EngineerManual::shiftCPress()
{
    toward_change_mode_ = 0;
    runStepQueue("GROUND_GIMBAL");
    ROS_INFO("enter gimbal GROUND_GIMBAL");
}
void EngineerManual::shiftZPress()
{
    toward_change_mode_ = 0;
    runStepQueue("REVERSAL_GIMBAL");
    ROS_INFO("enter gimbal REVERSAL_GIMBAL");
}
void EngineerManual::shiftVPress()
{
    //gimbal
    gimbal_mode_ = RATE;
    ROS_INFO("MANUAL_VIEW");
}

void EngineerManual::shiftVRelease()
{
    //gimbal
    gimbal_mode_ = DIRECT;
    ROS_INFO("DIRECT");
}

void EngineerManual::shiftBPress()
{
    toward_change_mode_ = 1;
    runStepQueue("BACK_GIMBAL");
    ROS_INFO("enter gimbal BACK_GIMBAL");
}

void EngineerManual::judgeReversal(double translate_err,int reversal_dot,bool is_ready,ros::Duration period) {
    double translate_dif{}, threshold{};
    //reversal roll
    switch (reversal_dot_) {
        case 2: {
            reversal_state_ = "F_R";
            if (translate_err > threshold && is_ready == 0) {
                reversal_command_sender_->visionReversal(0, 0, translate_err, period);
            } else(translate_err < threshold && is_ready == 0)
            {
                reversal_command_sender_->setGroupVel(0.5, 0, 0.);
            }
        }
        case 3: {
            reversal_state_ = "B_L";
            reversal_command_sender_->visionReversal(0, 0, translate_err, period);
        }
    }
    //reversal pitch to ready
    switch (reversal_dot_) {
        case 0:
            break;
        case 1: {
            reversal_state_ = "F_L";
            break;
        }
        case 4: {
            reversal_state_ = "B_R";
            break;
        }

    }
}
} // namespace rm_manual
