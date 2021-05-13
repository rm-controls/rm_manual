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
template<typename T>
Manual<T>::Manual(ros::NodeHandle &node_handle):nh_(node_handle) {
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

}

template<typename T>
uint8_t Manual<T>::getShootSpeedCmd(int shoot_speed) {
  switch (shoot_speed) {
    case 10: return rm_msgs::ShootCmd::SPEED_10M_PER_SECOND;
    case 15: return rm_msgs::ShootCmd::SPEED_15M_PER_SECOND;
    case 16: return rm_msgs::ShootCmd::SPEED_16M_PER_SECOND;
    case 18: return rm_msgs::ShootCmd::SPEED_18M_PER_SECOND;
    case 30: return rm_msgs::ShootCmd::SPEED_30M_PER_SECOND;
    default: return 0;
  }
}
template<typename T>
void Manual<T>::loadParam() {
  if (rc_flag_) { // rc mode
    coefficient_x_ = getParam(nh_, "control_param/rc_param/coefficient_x", 3.5);
    coefficient_y_ = getParam(nh_, "control_param/rc_param/coefficient_y", 3.5);
    coefficient_angular_ = getParam(nh_, "control_param/rc_param/coefficient_angular", 6.0);
    coefficient_yaw_ = getParam(nh_, "control_param/rc_param/coefficient_yaw", 12.56);
    coefficient_pitch_ = getParam(nh_, "control_param/rc_param/coefficient_pitch", 12.56);
  }else{ // pc mode
    coefficient_x_ = getParam(nh_, "control_param/pc_param/coefficient_x", 3.5);
    coefficient_y_ = getParam(nh_, "control_param/pc_param/coefficient_y", 3.5);
    coefficient_angular_ = getParam(nh_, "control_param/pc_param/coefficient_angular", 6.0);
    coefficient_yaw_ = getParam(nh_, "control_param/pc_param/coefficient_yaw", 125.6);
    coefficient_pitch_ = getParam(nh_, "control_param/pc_param/coefficient_pitch", 125.6);
  }
}
template<typename T>
void Manual<T>::setChassis(uint8_t chassis_mode, double linear_x, double linear_y, double angular_z) {
  double accel_x = accel_x_;
  double accel_y = accel_y_;
  double accel_angular = accel_angular_;

  data_.chassis_cmd_.mode = chassis_mode;

  if (linear_x == 0.0)
    accel_x = accel_x_ * brake_multiple_;

  if (linear_y == 0.0)
    accel_y = accel_y_ * brake_multiple_;

  if (angular_z == 0.0)
    accel_angular = accel_angular_ * brake_multiple_;
  data_.chassis_cmd_.accel.linear.x = accel_x;
  data_.chassis_cmd_.accel.linear.y = accel_y;
  data_.chassis_cmd_.accel.angular.z = accel_angular;
  //power limit
  if (have_power_manager_) {//have power manger
    data_.chassis_cmd_.power_limit = data_.referee_->power_manager_data_.parameters[1];
  } else if (!(have_power_manager_) && data_.referee_->is_open_) {//do not have power manger and use referee data
    data_.chassis_cmd_.power_limit = data_.referee_->referee_data_.game_robot_status_.chassis_power_limit;
    if (data_.chassis_cmd_.power_limit > 120) data_.chassis_cmd_.power_limit = 120;
  } else {//use safety power
    data_.chassis_cmd_.power_limit = safety_power_;
  }

  data_.cmd_vel_.linear.x = linear_x * coefficient_x_;
  data_.cmd_vel_.linear.y = linear_y * coefficient_y_;
  data_.cmd_vel_.angular.z = angular_z * coefficient_angular_;

  data_.vel_cmd_pub_.publish(data_.cmd_vel_);
  data_.chassis_cmd_pub_.publish(data_.chassis_cmd_);
}

template<typename T>
void Manual<T>::setGimbal(uint8_t gimbal_mode, double rate_yaw, double rate_pitch,
                         uint8_t target_id, double bullet_speed) {
  data_.gimbal_cmd_.mode = gimbal_mode;

  data_.gimbal_cmd_.rate_yaw = rate_yaw * coefficient_yaw_;
  data_.gimbal_cmd_.rate_pitch = rate_pitch * coefficient_pitch_;

  data_.gimbal_cmd_.target_id = target_id;
  data_.gimbal_cmd_.bullet_speed = bullet_speed;
  data_.gimbal_cmd_pub_.publish(data_.gimbal_cmd_);
}

template<typename T>
void Manual<T>::setShoot(uint8_t shoot_mode, int shoot_speed, double shoot_hz, ros::Time now) {
  data_.shoot_cmd_.mode = shoot_mode;

  data_.shoot_cmd_.speed = getShootSpeedCmd(shoot_speed);
  data_.shoot_cmd_.hz = shoot_hz;
  data_.shoot_cmd_.stamp = now;

  data_.shooter_cmd_pub_.publish(data_.shoot_cmd_);
}
template<typename T>
void Manual<T>::setArm(double linear_x, double linear_y, double linear_z,
                      double angular_x, double angular_y, double angular_z, ros::Time now) {
  data_.arm_cmd_vel_.header.stamp = now;
  data_.arm_cmd_vel_.twist.linear.x = linear_x;
  data_.arm_cmd_vel_.twist.linear.y = linear_y;
  data_.arm_cmd_vel_.twist.linear.z = linear_z;
  data_.arm_cmd_vel_.twist.angular.x = angular_x;
  data_.arm_cmd_vel_.twist.angular.y = angular_y;
  data_.arm_cmd_vel_.twist.angular.z = angular_z;
  data_.engineer_vel_cmd_pub_.publish(data_.arm_cmd_vel_);
}
template<typename T>
void Manual<T>::ctrlQCallback(){

}
template<typename T>
void Manual<T>::ctrlWCallback(){

}
template<typename T>
void Manual<T>::qCallback(){

}
template<typename T>
void Manual<T>::wCallback(){

}
template<typename T>
void Manual<T>::eCallback(){

}
template<typename T>
void Manual<T>::rCallback(){

}
template<typename T>
void Manual<T>::aCallback(){

}
template<typename T>
void Manual<T>::sCallback(){

}
template<typename T>
void Manual<T>::dCallback(){

}
template<typename T>
void Manual<T>::fCallback(){

}
template<typename T>
void Manual<T>::gCallback(){

}
template<typename T>
void Manual<T>::zCallback(){

}
template<typename T>
void Manual<T>::xCallback(){

}
template<typename T>
void Manual<T>::cCallback(){

}
template<typename T>
void Manual<T>::vCallback(){

}
template<typename T>
void Manual<T>::shiftCallback(){

}

template<typename T>
void Manual<T>::run() {
  // run referee system
  data_.referee_->read();
  if(data_.dbus_data_.s_r == rm_msgs::DbusData::DOWN){
    passive_flag_ = true;
    raw_flag_ = false;
    rc_flag_ = true;
  }else if(data_.dbus_data_.s_r == rm_msgs::DbusData::MID){
    passive_flag_ = false;
    raw_flag_ = true;
    rc_flag_ = true;
  }else if(data_.dbus_data_.s_r == rm_msgs::DbusData::UP){
    passive_flag_ = true;
    raw_flag_ = false;
    rc_flag_ = false;
  }
  loadParam();
  if(rc_flag_){
    if(passive_flag_){

    }else{

    }
  }else{
    if(data_.dbus_data_.key_q){
      if(data_.dbus_data_.key_ctrl){
        this->ctrlQCallback();
      }
      else{
        this->qCallback();
      }
    }
    if(data_.dbus_data_.key_w){
      if(data_.dbus_data_.key_ctrl){
        this->ctrlWCallback();
      }
      else{
        this->wCallback();
      }
    }
    if(data_.dbus_data_.key_e){
      this->eCallback();
    }
    if(data_.dbus_data_.key_r){
      this->rCallback();
    }
    if(data_.dbus_data_.key_a){
      this->aCallback();
    }
    if(data_.dbus_data_.key_s){
      this->sCallback();
    }
    if(data_.dbus_data_.key_d){
      this->dCallback();
    }
    if(data_.dbus_data_.key_f){
      this->fCallback();
    }
    if(data_.dbus_data_.key_g){
      this->gCallback();
    }
    if(data_.dbus_data_.key_z){
      this->zCallback();
    }
    if(data_.dbus_data_.key_x){
      this->xCallback();
    }
    if(data_.dbus_data_.key_c){
      this->cCallback();
    }
    if(data_.dbus_data_.key_v){
      this->vCallback();
    }
    if(data_.dbus_data_.key_shift){
      this->shiftCallback();
    }
  }

  data_.vel_cmd_pub_.publish(data_.cmd_vel_);
  data_.chassis_cmd_pub_.publish(data_.chassis_cmd_);
  data_.gimbal_cmd_pub_.publish(data_.gimbal_cmd_);
  data_.shooter_cmd_pub_.publish(data_.shoot_cmd_);
  data_.engineer_vel_cmd_pub_.publish(data_.arm_cmd_vel_);
}

// RobotRunner a template
template
class Manual<float>;
template
class Manual<double>;
