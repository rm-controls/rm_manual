//
// Created by peter on 2021/5/24.
//

#ifndef RM_MANUAL_REFEREE_UI_H_
#define RM_MANUAL_REFEREE_UI_H_

#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <rm_common/referee/referee.h>

namespace rm_manual {
class UiBase {
 public:
  UiBase(rm_common::Referee *referee) : referee_(referee) {}
  void setOperateType(GraphicOperateType operate_type) { operate_type_ = operate_type; }
 protected:
  rm_common::Referee *referee_;
  GraphicOperateType operate_type_ = ADD;
  GraphicColorType color_ = YELLOW;
  std::string display_info_;
  int picture_id_, picture_x_, picture_y_;
};

class UiManual : public UiBase {
 public:
  UiManual(rm_common::Referee *referee) : UiBase(referee) {};
  virtual void display(uint8_t mode, bool flag = false) {
    if (operate_type_ != ADD && last_mode_ == mode && last_flag_ == flag) return;
    color_ = flag ? ORANGE : YELLOW;
    getInfo(mode);
    referee_->drawString(picture_x_, picture_y_, picture_id_, display_info_, color_, operate_type_);
    last_mode_ = mode;
    last_flag_ = flag;
  }
  virtual void display(bool flag) {
    if (operate_type_ != ADD && last_flag_ == flag) return;
    getInfo(flag);
    referee_->drawString(picture_x_, picture_y_, picture_id_, display_info_, YELLOW, operate_type_);
    last_flag_ = flag;
  }
 protected:
  virtual void getInfo(uint8_t mode) {};
  virtual void getInfo(bool flag) {};
  uint8_t last_mode_;
  bool last_flag_ = false;
};

class UiChassis : public UiManual {
 public:
  UiChassis(rm_common::Referee *referee) : UiManual(referee) {
    last_mode_ = rm_msgs::ChassisCmd::FOLLOW;
    picture_id_ = 4;
    picture_x_ = 1470;
    picture_y_ = 790;
  }
 protected:
  void getInfo(uint8_t mode) override {
    if (mode == rm_msgs::ChassisCmd::FOLLOW) display_info_ = "chassis:follow";
    else if (mode == rm_msgs::ChassisCmd::GYRO) display_info_ = "chassis:gyro";
    else if (mode == rm_msgs::ChassisCmd::TWIST) display_info_ = "chassis:twist";
  }
};

class UiGimbal : public UiManual {
 public:
  UiGimbal(rm_common::Referee *referee) : UiManual(referee) {
    last_mode_ = rm_msgs::GimbalCmd::RATE;
    picture_id_ = 5;
    picture_x_ = 1470;
    picture_y_ = 740;
  }
 protected:
  void getInfo(uint8_t mode) override {
    if (mode == rm_msgs::GimbalCmd::RATE) display_info_ = "gimbal:rate";
    else if (mode == rm_msgs::GimbalCmd::TRACK) display_info_ = "gimbal:track";
  }
};

class UiShooter : public UiManual {
 public:
  UiShooter(rm_common::Referee *referee) : UiManual(referee) {
    last_mode_ = rm_msgs::ShootCmd::STOP;
    picture_id_ = 6;
    picture_x_ = 1470;
    picture_y_ = 690;
  }
 protected:
  void getInfo(uint8_t mode) override {
    if (mode == rm_msgs::ShootCmd::STOP) display_info_ = "shooter:stop";
    else if (mode == rm_msgs::ShootCmd::READY) display_info_ = "shooter:ready";
    else if (mode == rm_msgs::ShootCmd::PUSH) display_info_ = "shooter:push";
  }
};

class UiTarget : public UiManual {
 public:
  UiTarget(rm_common::Referee *referee) : UiManual(referee) {
    picture_id_ = 7;
    picture_x_ = 1470;
    picture_y_ = 640;
  }
 protected:
  void getInfo(bool base_only) override {
    if (base_only) display_info_ = "target:base";
    else display_info_ = "target:all";
  }
};

class UiAuto : public UiBase {
 public:
  UiAuto(rm_common::Referee *referee) : UiBase(referee) {};
  virtual void display(const ros::Time &time) {
    if (operate_type_ != ADD && time - last_update_ < ros::Duration(0.5)) return;
    getInfo();
    referee_->drawString(picture_x_, picture_y_, picture_id_, display_info_, color_, operate_type_);
    last_update_ = time;
  }
 protected:
  virtual void getInfo() {};
  ros::Time last_update_ = ros::Time::now();
};

class UiCapacitor : public UiAuto {
 public:
  UiCapacitor(rm_common::Referee *referee) : UiAuto(referee) {
    picture_id_ = 8;
    picture_x_ = 910;
    picture_y_ = 100;
  };
 protected:
  void getInfo() override {
    float cap_power = referee_->super_capacitor_.getCapPower() * 100;
    if (cap_power >= 60) color_ = GREEN;
    else if (cap_power < 60 && cap_power >= 30) color_ = YELLOW;
    else color_ = ORANGE;
    char power_string[30] = {' '};
    sprintf(power_string, "Cap: %1.0f%%", cap_power);
    display_info_ = power_string;
  }
};

class UiArmor : public UiAuto {
 public:
  UiArmor(rm_common::Referee *referee, int armor_id) : UiAuto(referee) { armor_id_ = armor_id; }
  void display(const ros::Time &time) override {
    if (referee_->referee_data_.robot_hurt_.hurt_type_ == 0x00
        && referee_->referee_data_.robot_hurt_.armor_id_ == armor_id_) {
      updateArmorPosition();
      referee_->drawCircle(picture_x_, picture_y_, 50, armor_id_, color_, ADD);
      last_update_ = time;
      delete_flag_ = false;
      referee_->referee_data_.robot_hurt_.hurt_type_ = 0x01;
    }
    if (!delete_flag_ && time - last_update_ > ros::Duration(0.5)) {
      referee_->drawCircle(0, 0, 0, armor_id_, YELLOW, DELETE);
      delete_flag_ = true;
    }
  }
 protected:
  void updateArmorPosition() {
    geometry_msgs::TransformStamped yaw2baselink;
    double roll, pitch, yaw;
    try { yaw2baselink = tf_.lookupTransform("yaw", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) {}
    quatToRPY(yaw2baselink.transform.rotation, roll, pitch, yaw);
    if (armor_id_ == 0 || armor_id_ == 2) {
      picture_x_ = (int) (960 + 340 * sin(armor_id_ * M_PI_2 + yaw));
      picture_y_ = (int) (540 + 340 * cos(armor_id_ * M_PI_2 + yaw));
    } else {
      picture_x_ = (int) (960 + 340 * sin(-armor_id_ * M_PI_2 + yaw));
      picture_y_ = (int) (540 + 340 * cos(-armor_id_ * M_PI_2 + yaw));
    }
  }
  int armor_id_;
  bool delete_flag_ = false;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_{};
};
} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
