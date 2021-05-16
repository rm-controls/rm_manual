//
// Created by peter on 2020/12/3.
//

#include "rm_manual/manual_common.h"
/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 * @tparam T
 * @param nh
 */

Manual::Manual(ros::NodeHandle &node_handle) : nh_(node_handle) {
  tf_listener_ = new tf2_ros::TransformListener(tf_);

  data_.init(nh_);
  accel_x_ = getParam(nh_, "control_param/accel_x", 5.0);
  accel_y_ = getParam(nh_, "control_param/accel_y", 5.0);
  accel_angular_ = getParam(nh_, "control_param/accel_angular", 5.0);
  brake_multiple_ = getParam(nh_, "control_param/brake_multiple", 2);
  expect_shoot_hz_ = getParam(nh_, "control_param/expect_shoot_hz", 5.0);
  safe_shoot_hz_ = getParam(nh_, "control_param/safe_shoot_hz", 2.0);
  safe_shoot_speed_ = getParam(nh_, "control_param/safe_shoot_speed", 10.0);
  gimbal_error_limit_ = getParam(nh_, "control_param/gimbal_error_limit", 0.5);
  safety_power_ = getParam(nh_, "power_limit/safety_power", 50);
  have_power_manager_ = getParam(nh_, "power_limit/have_power_manager", false);
  actual_shoot_speed_ = safe_shoot_speed_;
  ultimate_shoot_speed_ = safe_shoot_speed_;
  controller_manager_ = new ControllerManager(node_handle);
  current_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
  data_.dbus_data_.stamp = ros::Time::now();
  controller_manager_->loadAllControllers();
}

void Manual::run() {
  ros::Time now = ros::Time::now();
  // run referee system
  data_.referee_->read();
  loadParam();
  if (remote_control_is_open_ && ((now - data_.dbus_data_.stamp).toSec() > 0.1)) {
      this->remoteControlTurnOff();
      remote_control_is_open_ = false;
      ROS_INFO("remote control turn off");
  } else if (!remote_control_is_open_ && ((now - data_.dbus_data_.stamp).toSec() < 0.1)) {
      this->remoteControlTurnOn();
      remote_control_is_open_ = true;
      ROS_INFO("remote control turn on");
  }
  if (data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN) {
    this->rightSwitchDown();
  } else if (data_.dbus_data_.s_r == rm_msgs::DbusData::MID) {
    if (data_.dbus_data_.s_l == rm_msgs::DbusData::DOWN) {
      this->leftSwitchDown();
    } else if (data_.dbus_data_.s_l == rm_msgs::DbusData::MID) {
      this->leftSwitchMid();
    } else if (data_.dbus_data_.s_l == rm_msgs::DbusData::UP) {
      this->leftSwitchUp();
    }
    this->rightSwitchMid();
  } else if (data_.dbus_data_.s_r == rm_msgs::DbusData::UP) {
    this->rightSwitchUp();
  }
  if (data_.dbus_data_.key_f) {
    this->fPress(now - last_press_f_);
  }
  if (data_.dbus_data_.key_g) {
    this->gPress(now - last_press_g_);
  }
  if (data_.dbus_data_.key_r) {
    this->sPress(now - last_press_s_);
  }
  if (data_.dbus_data_.key_q) {
    if (data_.dbus_data_.key_ctrl) {
      this->ctrlQPress();
    } else {
      this->qPress(now - last_press_q_);
    }
  }
  if (data_.dbus_data_.key_w) {
    if (data_.dbus_data_.key_ctrl) {
      this->ctrlWPress();
    } else {
      this->wPress(now - last_press_w_);
    }
  }
  if (data_.dbus_data_.key_e) {
    this->ePress(now - last_press_e_);
  }
  if (data_.dbus_data_.key_r) {
    this->rPress(now - last_press_r_);
  }
  if (data_.dbus_data_.key_a) {
    this->aPress(now - last_press_a_);
  }

  if (data_.dbus_data_.key_d) {
    this->dPress(now - last_press_d_);
  }
  if (data_.dbus_data_.key_z) {
    this->zPress(now - last_press_z_);
  }
  if (data_.dbus_data_.key_x) {
    this->xPress(now - last_press_x_);
  }
  if (data_.dbus_data_.key_c) {
    this->cPress(now - last_press_c_);
  }
  if (data_.dbus_data_.key_v) {
    this->vPress(now - last_press_v_);
  }
  if (data_.dbus_data_.p_l) {
    if (data_.dbus_data_.p_r) {
      this->mouseLeftRightPress(now - last_press_mouse_right_left_);
    } else {
      this->mouseLeftPress(now - last_press_mouse_left_);
    }
  }
  if (data_.dbus_data_.p_r) {
    if (!data_.dbus_data_.p_l)
      this->mouseRightPress(now - last_press_mouse_right_);
  }
  if (data_.dbus_data_.key_shift) {
    this->shiftPress(now - last_press_shift_);
  }
  data_.chassis_cmd_.mode = current_chassis_mode_;
  if (emergency_stop_) {
    data_.chassis_cmd_.mode = rm_msgs::ChassisCmd::PASSIVE;
    data_.gimbal_cmd_.mode = rm_msgs::GimbalCmd::PASSIVE;
    data_.shoot_cmd_.mode = rm_msgs::ShootCmd::STOP;
  }
  powerLimit();
  data_.vel_cmd_pub_.publish(data_.cmd_vel_);
  data_.chassis_cmd_pub_.publish(data_.chassis_cmd_);
  data_.gimbal_cmd_pub_.publish(data_.gimbal_cmd_);
  data_.shooter_cmd_pub_.publish(data_.shoot_cmd_);
  data_.engineer_vel_cmd_pub_.publish(data_.arm_cmd_vel_);

}
void Manual::powerLimit() {
  if (have_power_manager_) {//have power manger
    data_.chassis_cmd_.power_limit = data_.referee_->power_manager_data_.parameters[1];
  } else if (!(have_power_manager_) && data_.referee_->is_open_) {//do not have power manger and use referee data
    data_.chassis_cmd_.power_limit = data_.referee_->referee_data_.game_robot_status_.chassis_power_limit;
    if (data_.chassis_cmd_.power_limit > 120) data_.chassis_cmd_.power_limit = 120;
  } else {//use safety power
    data_.chassis_cmd_.power_limit = safety_power_;
  }
}
uint8_t Manual::getShootSpeedCmd(int shoot_speed) {
  switch (shoot_speed) {
    case 10: return rm_msgs::ShootCmd::SPEED_10M_PER_SECOND;
    case 15: return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;
    case 16: return rm_msgs::ShootCmd::SPEED_16M_PER_SECOND;
    case 18: return rm_msgs::ShootCmd::SPEED_18M_PER_SECOND;
    case 30: return rm_msgs::ShootCmd::SPEED_30M_PER_SECOND;
    default: return 0;
  }
}

void Manual::loadParam() {
  if (rc_flag_) { // rc mode
    coefficient_x_ = getParam(nh_, "control_param/rc_param/coefficient_x", 3.5);
    coefficient_y_ = getParam(nh_, "control_param/rc_param/coefficient_y", 3.5);
    coefficient_angular_ = getParam(nh_, "control_param/rc_param/coefficient_angular", 6.0);
    coefficient_yaw_ = getParam(nh_, "control_param/rc_param/coefficient_yaw", 12.56);
    coefficient_pitch_ = getParam(nh_, "control_param/rc_param/coefficient_pitch", 12.56);
  } else { // pc mode
    coefficient_x_ = getParam(nh_, "control_param/pc_param/coefficient_x", 3.5);
    coefficient_y_ = getParam(nh_, "control_param/pc_param/coefficient_y", 3.5);
    coefficient_angular_ = getParam(nh_, "control_param/pc_param/coefficient_angular", 6.0);
    coefficient_yaw_ = getParam(nh_, "control_param/pc_param/coefficient_yaw", 125.6);
    coefficient_pitch_ = getParam(nh_, "control_param/pc_param/coefficient_pitch", 125.6);
  }
}

void Manual::leftSwitchDown() {
  emergency_stop_ = false;
  setGimbal(rm_msgs::GimbalCmd::RATE, -data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y, 0, 0.0);
  setShoot(rm_msgs::ShootCmd::STOP, ultimate_shoot_speed_, 0.0, ros::Time::now());
}

void Manual::leftSwitchMid() {
  uint8_t target_id;
  double shoot_hz = 0;
  emergency_stop_ = false;
  data_.target_cost_function_->input(data_.track_data_array_,
                                     data_.referee_->referee_data_.game_robot_hp_,
                                     false);
  target_id = data_.target_cost_function_->output();
  ultimate_shoot_speed_ = data_.referee_->getUltimateBulletSpeed(ultimate_shoot_speed_);
  if (target_id == 0) {
    if (last_target_id_ != 0)
      setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, last_target_id_, ultimate_shoot_speed_);
    else
      setGimbal(rm_msgs::GimbalCmd::RATE, -data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y, 0, 0.0);
  } else {
    last_target_id_ = target_id;
    setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, target_id, ultimate_shoot_speed_);
  }
  setShoot(rm_msgs::ShootCmd::READY, ultimate_shoot_speed_, shoot_hz, ros::Time::now());
}

void Manual::leftSwitchUp() {
  uint8_t target_id;
  double shoot_hz = 0;
  emergency_stop_ = false;
  data_.target_cost_function_->input(data_.track_data_array_,
                                     data_.referee_->referee_data_.game_robot_hp_,
                                     false);
  target_id = data_.target_cost_function_->output();
  actual_shoot_speed_ = data_.referee_->getActualBulletSpeed(actual_shoot_speed_);

  if (target_id == 0) {
    if (last_target_id_ != 0)
      setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, last_target_id_, actual_shoot_speed_);
    else
      setGimbal(rm_msgs::GimbalCmd::RATE, -data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y, 0, 0.0);
  } else {
    last_target_id_ = target_id;
    setGimbal(rm_msgs::GimbalCmd::TRACK, 0.0, 0.0, target_id, actual_shoot_speed_);
  }
  data_.shooter_heat_limit_->input(data_.referee_, expect_shoot_hz_, safe_shoot_hz_);
  shoot_hz = data_.shooter_heat_limit_->output();
  if (data_.gimbal_des_error_.error >= gimbal_error_limit_) { // check  error
    setShoot(rm_msgs::ShootCmd::READY, ultimate_shoot_speed_, shoot_hz, ros::Time::now());
  } else {
    setShoot(rm_msgs::ShootCmd::PUSH, ultimate_shoot_speed_, shoot_hz, ros::Time::now());
  }

}

void Manual::rightSwitchDown() {
  emergency_stop_ = true;
}
void Manual::rightSwitchMid() {
  emergency_stop_ = false;
  rc_flag_ = true;
  if (data_.dbus_data_.wheel) { // enter gyro
    current_chassis_mode_ = rm_msgs::ChassisCmd::GYRO;
  } else { // enter follow
    current_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
  }
  setChassis(data_.dbus_data_.ch_r_y, -data_.dbus_data_.ch_r_x, data_.dbus_data_.wheel);
}
void Manual::rightSwitchUp() {
  emergency_stop_ = false;
  rc_flag_ = false;
}
void Manual::ctrlQPress() {
  emergency_stop_ = true;
}

void Manual::ctrlWPress() {
  emergency_stop_ = false;
}

void Manual::qPress(ros::Duration period) {
  if (period.toSec() > 0.5)
    is_burst_ = !is_burst_;
}

void Manual::wPress(ros::Duration period) {
  data_.chassis_cmd_.mode = current_chassis_mode_;
  data_.cmd_vel_.linear.x = 1.0;
  data_.chassis_cmd_.accel.linear.x = accel_x_;
}

void Manual::ePress(ros::Duration period) {

}

void Manual::rPress(ros::Duration period) {
  current_chassis_mode_ = rm_msgs::ChassisCmd::TWIST;
}

void Manual::aPress(ros::Duration period) {
  data_.chassis_cmd_.mode = current_chassis_mode_;
  data_.cmd_vel_.linear.y = 1.0;
  data_.chassis_cmd_.accel.linear.y = accel_y_;
}

void Manual::sPress(ros::Duration period) {
  data_.chassis_cmd_.mode = current_chassis_mode_;
  data_.cmd_vel_.linear.x = -1.0;
  data_.chassis_cmd_.accel.linear.x = accel_x_;
}

void Manual::dPress(ros::Duration period) {
  data_.chassis_cmd_.mode = current_chassis_mode_;
  data_.cmd_vel_.linear.y = -1.0;
  data_.chassis_cmd_.accel.linear.y = accel_y_;
}

void Manual::fPress(ros::Duration period) {
  current_chassis_mode_ = rm_msgs::ChassisCmd::FOLLOW;
}

void Manual::gPress(ros::Duration period) {
  current_chassis_mode_ = rm_msgs::ChassisCmd::GYRO;
}

void Manual::zPress(ros::Duration period) {

}

void Manual::xPress(ros::Duration period) {

}

void Manual::cPress(ros::Duration period) {

}

void Manual::vPress(ros::Duration period) {

}
void Manual::bPress(ros::Duration period) {
  if (period.toSec() > 0.5) {
    only_attack_base_ = !only_attack_base_;
  }
}
void Manual::shiftPress(ros::Duration period) {
  data_.cmd_vel_.linear.x *= 2.0;
  data_.cmd_vel_.linear.y *= 2.0;
}

void Manual::mouseLeftPress(ros::Duration period) {
  data_.shoot_cmd_.mode = rm_msgs::ShootCmd::PUSH;
  if (is_burst_) { // ignore shooter heat limit
    data_.shoot_cmd_.hz = expect_shoot_hz_;
  } else {
    data_.shooter_heat_limit_->input(data_.referee_, expect_shoot_hz_, safe_shoot_hz_);
    data_.shoot_cmd_.hz = data_.shooter_heat_limit_->output();
  }
}

void Manual::mouseRightPress(ros::Duration period) {
  int target_id = 0;
  data_.target_cost_function_->input(data_.track_data_array_,
                                     data_.referee_->referee_data_.game_robot_hp_,
                                     only_attack_base_);
  target_id = data_.target_cost_function_->output();
  if (target_id == 0) {
    data_.gimbal_cmd_.mode = rm_msgs::GimbalCmd::RATE;
  } else {
    data_.gimbal_cmd_.target_id = target_id;
    data_.gimbal_cmd_.mode = rm_msgs::GimbalCmd::TRACK;
  }
}

void Manual::mouseLeftRightPress(ros::Duration period) {
  ros::Time now = ros::Time::now();
  int target_id = 0;
  data_.target_cost_function_->input(data_.track_data_array_,
                                     data_.referee_->referee_data_.game_robot_hp_,
                                     only_attack_base_);
  target_id = data_.target_cost_function_->output();
  if (target_id == 0) {
    data_.gimbal_cmd_.mode = rm_msgs::GimbalCmd::RATE;
  } else {
    data_.gimbal_cmd_.target_id = target_id;
    data_.gimbal_cmd_.mode = rm_msgs::GimbalCmd::TRACK;
  }
  if (now - data_.gimbal_des_error_.stamp > ros::Duration(1.0)) { // check time stamp
    data_.gimbal_des_error_.error = 0;
    ROS_WARN("The time stamp of gimbal track error is too old");
  }
  if (data_.gimbal_des_error_.error >= this->gimbal_error_limit_) { // check  error
    data_.shoot_cmd_.mode = rm_msgs::ShootCmd::READY;
  } else {
    data_.shoot_cmd_.mode = rm_msgs::ShootCmd::PUSH;
  }
}

void Manual::setChassis(double linear_x, double linear_y, double angular_z) {
  double accel_x = accel_x_;
  double accel_y = accel_y_;
  double accel_angular = accel_angular_;

  if (angular_z == 0.0)
    accel_angular = accel_angular_ * brake_multiple_;
  data_.chassis_cmd_.accel.linear.x = accel_x;
  data_.chassis_cmd_.accel.linear.y = accel_y;
  data_.chassis_cmd_.accel.angular.z = accel_angular;

  data_.cmd_vel_.linear.x = linear_x * coefficient_x_;
  data_.cmd_vel_.linear.y = linear_y * coefficient_y_;
  data_.cmd_vel_.angular.z = angular_z * coefficient_angular_;

  //data_.vel_cmd_pub_.publish(data_.cmd_vel_);
  //data_.chassis_cmd_pub_.publish(data_.chassis_cmd_);
}

void Manual::setGimbal(uint8_t gimbal_mode, double rate_yaw, double rate_pitch,
                       uint8_t target_id, double bullet_speed) {
  data_.gimbal_cmd_.mode = gimbal_mode;

  data_.gimbal_cmd_.rate_yaw = rate_yaw * coefficient_yaw_;
  data_.gimbal_cmd_.rate_pitch = rate_pitch * coefficient_pitch_;

  data_.gimbal_cmd_.target_id = target_id;
  data_.gimbal_cmd_.bullet_speed = bullet_speed;
  //data_.gimbal_cmd_pub_.publish(data_.gimbal_cmd_);
}

void Manual::setShoot(uint8_t shoot_mode, int shoot_speed, double shoot_hz, ros::Time now) {
  data_.shoot_cmd_.mode = shoot_mode;
  data_.shoot_cmd_.speed = getShootSpeedCmd(shoot_speed);
  data_.shoot_cmd_.hz = shoot_hz;
  data_.shoot_cmd_.stamp = now;

  //data_.shooter_cmd_pub_.publish(data_.shoot_cmd_);
}

void Manual::setArm(double linear_x, double linear_y, double linear_z,
                    double angular_x, double angular_y, double angular_z, ros::Time now) {
  data_.arm_cmd_vel_.header.stamp = now;
  data_.arm_cmd_vel_.twist.linear.x = linear_x;
  data_.arm_cmd_vel_.twist.linear.y = linear_y;
  data_.arm_cmd_vel_.twist.linear.z = linear_z;
  data_.arm_cmd_vel_.twist.angular.x = angular_x;
  data_.arm_cmd_vel_.twist.angular.y = angular_y;
  data_.arm_cmd_vel_.twist.angular.z = angular_z;
  //data_.engineer_vel_cmd_pub_.publish(data_.arm_cmd_vel_);
}

void Manual::remoteControlTurnOff() {
  controller_manager_->stopAllControllers();
}
void Manual::remoteControlTurnOn() {
  controller_manager_->startAllControllers();
}
// RobotRunner a template

class Manual;
