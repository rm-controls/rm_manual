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
    gyro_low_scale_ = 0.15;
  if (!vel_nh.getParam("gyro_low_low_scale", gyro_low_low_scale_))
    gyro_low_low_scale_ = 0.05;
  if (!vel_nh.getParam("exchange_speed_scale", exchange_speed_scale_))
    exchange_speed_scale_ = 0.12;
  // Ui
  exchange_sub_ = nh.subscribe<rm_msgs::ExchangerMsg>("/pnp_publisher", 1, &EngineerManual::exchangeCallback, this);
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
  // Auto Search
  ros::NodeHandle nh_auto_search(nh, "auto_search");
  XmlRpc::XmlRpcValue pitch_value, yaw_value;
  nh_auto_search.getParam("pitch", pitch_value);
  pitch_min_ = (double)(pitch_value[0]);
  pitch_max_ = (double)(pitch_value[1]);
  nh_auto_search.getParam("yaw", yaw_value);
  yaw_min_ = (double)(yaw_value[0]);
  yaw_max_ = (double)(yaw_value[1]);
  if (!nh_auto_search.getParam("search_angle", search_angle_))
    search_angle_ = 1.57;
  if (!nh_auto_search.getParam("chassis_search_vel", chassis_search_vel_))
    chassis_search_vel_ = 0.18;
  if (!nh_auto_search.getParam("gimbal_search_vel", gimbal_search_vel_))
    gimbal_search_vel_ = 0.2;
  if (!nh_auto_search.getParam("chassis_pos_tolerance", chassis_pos_tolerance_))
    chassis_pos_tolerance_ = 0.02;
  if (!nh_auto_search.getParam("chassis_angular_tolerance", chassis_angular_tolerance_))
    chassis_angular_tolerance_ = 0.02;
  if (!nh_auto_search.getParam("x_start_value", x_start_value_))
    x_start_value_ = 1.1;
  if (!nh_auto_search.getParam("pre_yaw_scales", pre_yaw_scales_))
    pre_yaw_scales_ = 0.5;
  store_chassis_search_vel_ = chassis_search_vel_;
  store_gimbal_search_vel_ = gimbal_search_vel_;
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
  shift_r_event_.setActiveHigh(boost::bind(&EngineerManual::shiftRPressing, this));
  shift_r_event_.setFalling(boost::bind(&EngineerManual::shiftRRelease, this));
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
  if (servo_mode_ == SERVO)
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

void EngineerManual::shiftRPressing()
{
  autoExchange();
  is_enter_auto_ = true;
}

void EngineerManual::autoExchange()
{
  switch (auto_process_)
  {
    case FIND:
      find();
      break;
    case PRE_ADJUST:
      preAdjust();
      break;
    case MOVE:
      move();
      break;
    case POST_ADJUST:
      postAdjust();
      break;
    case FINISH:
      finish();
      break;
  }
}

void EngineerManual::shiftRRelease()
{
  auto_process_ = FIND;
  chassis_error_pos_ = 1e10;
  chassis_error_yaw_ = 1e10;
  move_times_ = 0;
  is_move_start_ = false;
  move_finish_ = false;
  is_recorded_ = false;
  is_search_finish_ = false;
  is_pre_adjust_finish_ = false;
  is_move_finish_ = false;
  is_post_adjust_finish_ = false;
  is_enter_auto_ = false;
  pre_arm_start_ = false;
}

void EngineerManual::find()
{
  {
    if (!is_search_finish_)
    {
      autoSearch(false, true);
      ROS_INFO_STREAM("FIND");
    }
    else
    {
      // runStepQueue("SIDE_GIMBAL");
      ROS_INFO("ready to switch state");
      auto_process_ = PRE_ADJUST;
    }
  }
}
void EngineerManual::preAdjust()
{
  if (!pre_arm_start_)
  {
    double roll, pitch, yaw;
    exchange2base_ = tf_buffer_.lookupTransform("base_link", "exchanger", ros::Time(0));
    quatToRPY(exchange2base_.transform.rotation, roll, pitch, yaw);
    ROS_INFO_STREAM(yaw);
    if (yaw < 0.)
    {
      prefix_ = "EXCHANGE_AUTO_";
      root_ = "LF";
    }
    else
    {
      prefix_ = "EXCHANGE_AUTO_";
      root_ = "RT";
    }
    runStepQueue(prefix_ + root_);
    pre_arm_start_ = true;
  }
  if (move_finish_)
  {
    if (!is_pre_adjust_finish_)
    {
      autoPreAdjust();
      ROS_INFO_STREAM("PRE");
    }
    else
    {
      pre_arm_start_ = false;
      move_finish_ = false;
      auto_process_ = MOVE;
    }
  }
}

void EngineerManual::move()
{
  if (!is_move_finish_)
  {
    if (operating_mode_ == MANUAL && !is_move_start_)
    {
      autoMove();
      ROS_INFO_STREAM("MOVE");
    }
    if (move_finish_)
    {
      isArmFinish();
    }
    if (is_need_post_adjust_)
      auto_process_ = POST_ADJUST;
  }
  else
    auto_process_ = FINISH;
}

void EngineerManual::postAdjust()
{
  is_need_post_adjust_ = false;
  if (!is_post_adjust_finish_)
  {
    autoPostAdjust();
  }
  else
  {
    auto_process_ = MOVE;
  }
}

void EngineerManual::finish()
{
  move_finish_ = false;
  is_recorded_ = false;
  is_search_finish_ = false;
  pre_arm_start_ = false;
  is_pre_adjust_finish_ = false;
  is_post_adjust_finish_ = false;
  is_move_finish_ = false;
}

void EngineerManual::autoSearch(bool enable_chassis, bool enable_gimbal)
{
  if (enable_gimbal)
  {
    ros::Time time = ros::Time::now();
    if (!is_exchange_)
    {
      is_search_finish_ = false;
      lock_time_ = 0;
      base2yaw_ = tf_buffer_.lookupTransform("gimbal_base", "yaw", ros::Time(0));
      yaw2pitch_ = tf_buffer_.lookupTransform("yaw", "pitch", ros::Time(0));

      double yaw = yawFromQuat(base2yaw_.transform.rotation);
      double roll_temp, pitch, yaw_temp;
      quatToRPY(yaw2pitch_.transform.rotation, roll_temp, pitch, yaw_temp);
      if (yaw >= yaw_max_)
        yaw_direct_ = -1.;
      else if (yaw <= yaw_min_)
        yaw_direct_ = 1.;
      if (pitch >= pitch_max_)
        pitch_direct_ = -1.;
      else if (pitch <= pitch_min_)
        pitch_direct_ = 1.;
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
      gimbal_cmd_sender_->setRate(yaw_direct_ * gimbal_search_vel_, pitch_direct_ * gimbal_search_vel_);
      if (enable_chassis)
        vel_cmd_sender_->setAngularZVel(yaw_direct_ * chassis_search_vel_);
      if (chassis_search_vel_ < store_chassis_search_vel_ && gimbal_search_vel_ < store_gimbal_search_vel_)
      {
        chassis_search_vel_ = store_chassis_search_vel_;
        gimbal_search_vel_ = store_gimbal_search_vel_;
      }
    }
    else
    {
      lock_time_++;
      gimbal_cmd_sender_->setZero();
      vel_cmd_sender_->setZero();
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
      chassis_search_vel_ = store_chassis_search_vel_ / pow(10, lock_time_);
      gimbal_search_vel_ *= store_gimbal_search_vel_ / pow(10, lock_time_);
    }
    if (lock_time_ >= 10)
      is_search_finish_ = true;
    gimbal_cmd_sender_->sendCommand(time);
  }
}

void EngineerManual::autoPreAdjust()
{
  autoGimbalSearch();
  autoPreAdjustChassis();
}

void EngineerManual::autoPostAdjust()
{
  autoGimbalSearch();
  autoPostAdjustChassis();
}

void EngineerManual::autoMove()
{
  move_times_++;
  if (move_times_ == 1)
  {
    prefix_ = "EXCHANGE_AUTO_";
    root_ = "ONE";
  }
  else if (move_times_ == 2)
  {
    prefix_ = "ISLAND_GIMBAL";
    root_ = "";
    auto_process_ = FINISH;
  }
  is_move_start_ = true;
  runStepQueue(prefix_ + root_);
}

void EngineerManual::isArmFinish()
{
  exchange2stone_ = tf_buffer_.lookupTransform("tools_link", "exchanger", ros::Time(0));
  double error =
      sqrt(pow((exchange2stone_.transform.translation.x), 2) + pow((exchange2stone_.transform.translation.y), 2) +
           pow((exchange2stone_.transform.translation.z), 2));
  ROS_INFO_STREAM(error);
  if (error <= 0.001)
    is_move_finish_ = true;
  else
    is_need_post_adjust_ = true;
}
void EngineerManual::autoGimbalSearch()
{
  EngineerManual::autoSearch(false, true);
}

void EngineerManual::moveChassis()
{
  geometry_msgs::TransformStamped current;
  current = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));

  geometry_msgs::Vector3 error;
  error.x = chassis_target_.pose.position.x - current.transform.translation.x;
  error.y = chassis_target_.pose.position.y - current.transform.translation.y;

  double roll, pitch, yaw_current, yaw_goal, error_yaw;
  quatToRPY(current.transform.rotation, roll, pitch, yaw_current);
  quatToRPY(chassis_target_.pose.orientation, roll, pitch, yaw_goal);
  error_yaw = angles::shortest_angular_distance(yaw_current, yaw_goal);
  ROS_INFO_STREAM(error.x);
  ROS_INFO_STREAM(error.y);
  ROS_INFO_STREAM(error_yaw);
  geometry_msgs::Twist cmd_vel{};
  if (abs(error.x) >= chassis_pos_tolerance_ && chassis_original_target_.pose.position.x != 0)
    cmd_vel.linear.x = (error.x / abs(error.x)) * (0.05 + 0.1 * abs(error.x / 0.4));
  else
    cmd_vel.linear.x = 0;
  if (abs(error.y) >= chassis_pos_tolerance_ && chassis_original_target_.pose.position.y != 0)
    cmd_vel.linear.y = (error.y / abs(error.y)) * (0.05 + 0.1 * abs(error.y / 0.4));
  else
    cmd_vel.linear.y = 0;
  if ((abs(error_yaw) >= chassis_angular_tolerance_) && (chassis_error_pos_ < chassis_pos_tolerance_))
    cmd_vel.angular.z = (error_yaw / abs(error_yaw)) * (0.15 + abs(error_yaw / M_PI_2) * 0.35);
  else
    cmd_vel.angular.z = 0;
  chassis_error_yaw_ = abs(error_yaw);
  if (chassis_original_target_.pose.position.x == 0 && chassis_original_target_.pose.position.y != 0)
    chassis_error_pos_ = abs(error.y);
  else if (chassis_original_target_.pose.position.x != 0 && chassis_original_target_.pose.position.y == 0)
    chassis_error_pos_ = abs(error.x);
  else if (chassis_original_target_.pose.position.x != 0 && chassis_original_target_.pose.position.y != 0)
    chassis_error_pos_ = sqrt(pow(error.x, 2) + pow(error.y, 2));
  chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
  vel_cmd_sender_->setLinearXVel(cmd_vel.linear.x);
  vel_cmd_sender_->setLinearYVel(cmd_vel.linear.y);
  vel_cmd_sender_->setAngularZVel(cmd_vel.angular.z);
}

void EngineerManual::setChassisTarget(double goal_x, double goal_y, double goal_yaw)
{
  chassis_original_target_.pose.position.x = goal_x;
  chassis_original_target_.pose.position.y = goal_y;
  chassis_target_.pose.position.x = goal_x;
  chassis_target_.pose.position.y = goal_y;
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0, 0, goal_yaw);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
  chassis_target_.pose.orientation = quat_msg;
  chassis_original_target_.pose.orientation = quat_msg;
  tf2::doTransform(chassis_target_, chassis_target_, tf_buffer_.lookupTransform("map", "base_link", ros::Time(0)));
  chassis_error_pos_ = 1e10;
  chassis_error_yaw_ = 1e10;
}
void EngineerManual::autoPreAdjustChassis()
{
  is_pre_adjust_finish_ = true;
  if (!set_once_flag_)
  {
    geometry_msgs::TransformStamped exchange2base;
    double roll, pitch, yaw_error;
    exchange2base = tf_buffer_.lookupTransform("base_link", "exchanger", ros::Time(0));
    quatToRPY(exchange2base.transform.rotation, roll, pitch, yaw_error);
    // Set x offset
    double goal_x = exchange2base.transform.translation.x - x_start_value_;
    // double goal_y = exchange2base_.transform.translation.y;
    double goal_yaw = pre_yaw_scales_ * yaw_error;
    //      ROS_INFO_STREAM(exchange2base_);
    //      ROS_INFO_STREAM(goal_x);
    //      ROS_INFO_STREAM(goal_yaw);
    setChassisTarget(goal_x, 0., goal_yaw);
    set_once_flag_ = true;
  }
  if (!isChassisFinish())
    moveChassis();
  else
  {
    vel_cmd_sender_->setZero();
    set_once_flag_ = false;
    is_pre_adjust_finish_ = true;
  }
}
void EngineerManual::autoPostAdjustChassis()
{
  double goal_x, goal_y;
  link22base_ = tf_buffer_.lookupTransform("base_link", "link2", ros::Time(0));
  link32base_ = tf_buffer_.lookupTransform("base_link", "link3", ros::Time(0));
  if (!is_recorded_)
  {
    goal_x = link22base_.transform.translation.x;
    goal_y = link32base_.transform.translation.y;
    setChassisTarget(goal_x, goal_y, 0.);
    is_recorded_ = true;
    is_move_start_ = false;
    move_finish_ = false;
  }
  if (!is_move_start_)
  {
    runStepQueue("JOINT_TWO_AND_THREE_BACK");
    is_move_start_ = true;
  }
  if (move_finish_)
  {
    if (!isChassisFinish())
      moveChassis();
    else
    {
      is_recorded_ = false;
      is_move_start_ = false;
      move_finish_ = false;
      is_post_adjust_finish_ = true;
    }
  }
}

bool EngineerManual::isChassisFinish()
{
  return (((chassis_error_pos_ <= chassis_pos_tolerance_) && (chassis_error_yaw_ <= chassis_angular_tolerance_)));
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
  ChassisGimbalManual::updatePc(dbus_data);
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
    servo_command_sender_->sendCommand(time);
    speed_mode_ = LOW;
  }
  if (gimbal_mode_ == RATE)
    gimbal_cmd_sender_->sendCommand(time);
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
  //  if (operating_mode_ == MIDDLEWARE)
  //    action_client_.cancelAllGoals();
}

void EngineerManual::rightSwitchDownRise()
{
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
  EngineerManual::shiftVPress();
  ROS_INFO_STREAM("START CALIBRATE");
}

void EngineerManual::leftSwitchDownFall()
{
  runStepQueue("HOME_ZERO_STONE");
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
  reversal_motion_ = false;
  change_flag_ = true;
  if (result->finish)
    if (prefix_ + root_ == "EXCHANGE_AUTO_ONE" || prefix_ + root_ == "EXCHANGE_AUTO_TWO" ||
        prefix_ + root_ == "EXCHANGE_AUTO_THREE" || prefix_ + root_ == "JOINT_TWO_AND_THREE_BACK" ||
        prefix_ + root_ == "EXCHANGE_AUTO_LF" || prefix_ + root_ == "EXCHANGE_AUTO_RT")
      move_finish_ = true;
  if (prefix_ + root_ == "TWO_STONE_SMALL_ISLAND0")
    speed_mode_ = LOW;
  ROS_INFO("%i", result->finish);
  operating_mode_ = MANUAL;
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
  engineer_ui_.stone_num = std::to_string(stone_num_);
  engineer_ui_.joint_temperature = joint_temperature_;
  engineer_ui_.gripper_state = gripper_state_;
  if (servo_mode_ == SERVO)
    engineer_ui_.servo_mode = "SERVO";
  else
    engineer_ui_.servo_mode = "JOINT";
  engineer_ui_pub_.publish(engineer_ui_);
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
void EngineerManual::ctrlXPress()
{
  prefix_ = "NEW_LF_";
  root_ = "SMALL_ISLAND";
  speed_mode_ = LOW;
  runStepQueue("NEW_LF_SMALL_ISLAND");
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlEPress()
{
  prefix_ = "NEW_";
  root_ = "RT_SMALL_ISLAND";
  speed_mode_ = LOW;
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlWPress()
{
  prefix_ = "ARM_ADJUST";
  root_ = "";
  speed_mode_ = EXCHANGE;
  runStepQueue(prefix_ + root_);
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
  root_ = "NEW_SMALL_ISLAND";
  speed_mode_ = LOW;
  runStepQueue("NEW_SMALL_ISLAND");
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlSPress()
{
  prefix_ = "";
  root_ = "BIG_ISLAND";
  runStepQueue(prefix_ + root_);
  ROS_INFO("%s", (prefix_ + root_).c_str());
  speed_mode_ = LOW;
}

void EngineerManual::ctrlDPress()
{
  prefix_ = "GROUND_";
  root_ = "STONE0";
  runStepQueue(prefix_ + root_);
  speed_mode_ = LOW;
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlFPress()
{
  prefix_ = "EXCHANGE_";
  root_ = "WAIT";
  runStepQueue(prefix_ + root_);
  speed_mode_ = LOW;
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
  speed_mode_ = LOW;
}

void EngineerManual::ctrlZPress()
{
}

void EngineerManual::ctrlZPressing()
{
}

void EngineerManual::ctrlZRelease()
{
  if (is_exchange_)
  {
    prefix_ = "EXCHANGE_";
    root_ = "AUTO_ONE";
    runStepQueue(prefix_ + root_);
  }
  speed_mode_ = EXCHANGE;
}

void EngineerManual::ctrlQPress()
{
  prefix_ = "";
  root_ = "TWO_STONE_SMALL_ISLAND";
  runStepQueue("TWO_STONE_SMALL_ISLAND");
  ROS_INFO("%s", (prefix_ + root_).c_str());
}

void EngineerManual::ctrlCPress()
{
  runStepQueue("DELETE_SCENE");
  action_client_.cancelAllGoals();
}

void EngineerManual::shiftVPress()
{
  if (gripper_state_ == "open")
    runStepQueue("CLOSE_GRIPPER");
  else
  {
    runStepQueue("OPEN_GRIPPER");
  }
}

void EngineerManual::shiftVRelease()
{
}

void EngineerManual::ctrlBPress()
{
  initMode();
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
  if (speed_mode_ == NORMAL)
    vel_cmd_sender_->setAngularZVel(gyro_low_scale_);
  else if (speed_mode_ == LOW)
    vel_cmd_sender_->setAngularZVel(gyro_low_low_scale_);
  else if (speed_mode_ == FAST)
    vel_cmd_sender_->setAngularZVel(gyro_scale_);
  else if (speed_mode_ == EXCHANGE)
    vel_cmd_sender_->setAngularZVel(exchange_speed_scale_);
}

void EngineerManual::qRelease()
{
  vel_cmd_sender_->setAngularZVel(0);
}

void EngineerManual::ePressing()
{
  if (speed_mode_ == NORMAL)
    vel_cmd_sender_->setAngularZVel(-gyro_low_scale_);
  else if (speed_mode_ == LOW)
    vel_cmd_sender_->setAngularZVel(-gyro_low_low_scale_);
  else if (speed_mode_ == FAST)
    vel_cmd_sender_->setAngularZVel(-gyro_scale_);
  else if (speed_mode_ == EXCHANGE)
    vel_cmd_sender_->setAngularZVel(exchange_speed_scale_);
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
  if (stone_num_ != 3)
    stone_num_++;
  else
    stone_num_ = 0;
}

void EngineerManual::xPress()
{
  prefix_ = "ENGINEER_";
  root_ = "DRAG_CAR";
  runStepQueue(prefix_ + root_);
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
  reversal_command_sender_->setGroupValue(0., 0., -0.3, 0., 1.5, 0.);
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
  reversal_command_sender_->setGroupValue(0., 0., 3., 0., 0., 0.);
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
  speed_mode_ = FAST;
}
void EngineerManual::shiftRelease()
{
  speed_mode_ = NORMAL;
}
void EngineerManual::shiftFPress()
{
  runStepQueue("EXCHANGE_GIMBAL");
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
    servo_mode_ = SERVO;
    gimbal_mode_ = DIRECT;
    speed_mode_ = EXCHANGE;
    servo_reset_caller_->callService();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    action_client_.cancelAllGoals();
    chassis_cmd_sender_->getMsg()->command_source_frame = "fake_link5";
  }
}

void EngineerManual::initMode()
{
  servo_mode_ = JOINT;
  gimbal_mode_ = DIRECT;
  speed_mode_ = NORMAL;
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
}
void EngineerManual::shiftZPress()
{
  runStepQueue("ISLAND_GIMBAL");
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  ROS_INFO("enter gimbal REVERSAL_GIMBAL");
}
void EngineerManual::ctrlVPress()
{
  prefix_ = "TAKE_BLOCK";
  root_ = "";
  runStepQueue(prefix_ + root_);
}

void EngineerManual::ctrlVRelease()
{
}

void EngineerManual::shiftBPress()
{
  runStepQueue("SIDE_GIMBAL");
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
  ROS_INFO("enter gimbal BACK_GIMBAL");
}

void EngineerManual::shiftBRelease()
{
}

void EngineerManual::shiftXPress()
{
  runStepQueue("GROUND_GIMBAL");
  chassis_cmd_sender_->getMsg()->command_source_frame = "yaw";
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
  speed_mode_ = LOW;
  runStepQueue(prefix_ + root_);
}
}  // namespace rm_manual
