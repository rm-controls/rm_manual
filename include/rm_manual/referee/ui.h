//
// Created by peter on 2021/5/24.
//

#ifndef RM_MANUAL_REFEREE_UI_H_
#define RM_MANUAL_REFEREE_UI_H_

#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>
#include "rm_manual/referee/referee.h"

namespace rm_manual {
class UiBase {
 public:
  explicit UiBase(Referee *referee) : referee_(referee) {}
  void setOperateType(GraphicOperateType operate_type) { operate_type_ = operate_type; }
 protected:
  Referee *referee_;
  GraphicOperateType operate_type_ = UPDATE;
  GraphicColorType color_ = YELLOW;
  std::string display_title_{}, display_info_{};
  int picture_id_{}, picture_x_{}, picture_y_{};
  ros::Time init_time_;
  bool init_flag_ = false;
};

class UiManual : public UiBase {
 public:
  explicit UiManual(Referee *referee) : UiBase(referee) { picture_x_ = 150; };
  void display(const ros::Time &time, uint8_t mode, bool flag = false) {
    if (operate_type_ != ADD && last_mode_ == mode && last_flag_ == flag) return;
    color_ = flag ? ORANGE : YELLOW;
    getInfo(mode);
    if (operate_type_ == ADD) {
      if (!init_flag_) {
        init_time_ = time;
        init_flag_ = true;
      }
      if (time - init_time_ <= ros::Duration(0.2))
        referee_->drawString(picture_id_ + 20, picture_x_, picture_y_, display_title_, WHITE, ADD);
      else referee_->drawString(picture_id_, picture_x_ + 120, picture_y_, display_info_, color_, ADD);
    } else {
      referee_->drawString(picture_id_, picture_x_ + 120, picture_y_, display_info_, color_, operate_type_);
      last_mode_ = mode;
      last_flag_ = flag;
      init_flag_ = false;
    }
  }
 protected:
  virtual void getInfo(uint8_t mode) {};
  uint8_t last_mode_{};
  bool last_flag_ = false;
};

class UiChassis : public UiManual {
 public:
  explicit UiChassis(Referee *referee) : UiManual(referee) {
    last_mode_ = rm_msgs::ChassisCmd::FOLLOW;
    display_title_ = "chassis";
    picture_id_ = 4;
    picture_y_ = 790;
  }
 protected:
  void getInfo(uint8_t mode) override {
    if (mode == rm_msgs::ChassisCmd::FOLLOW) display_info_ = "follow";
    else if (mode == rm_msgs::ChassisCmd::GYRO) display_info_ = "gyro";
    else if (mode == rm_msgs::ChassisCmd::TWIST) display_info_ = "twist";
  }
};

class UiGimbal : public UiManual {
 public:
  explicit UiGimbal(Referee *referee) : UiManual(referee) {
    last_mode_ = rm_msgs::GimbalCmd::RATE;
    display_title_ = "gimbal";
    picture_id_ = 5;
    picture_y_ = 740;
  }
 protected:
  void getInfo(uint8_t mode) override {
    if (mode == rm_msgs::GimbalCmd::RATE) display_info_ = "rate";
    else if (mode == rm_msgs::GimbalCmd::DIRECT) display_info_ = "direct";
    else if (mode == rm_msgs::GimbalCmd::TRACK) display_info_ = "track";
  }
};

class UiShooter : public UiManual {
 public:
  explicit UiShooter(Referee *referee) : UiManual(referee) {
    last_mode_ = rm_msgs::ShootCmd::STOP;
    display_title_ = "shooter";
    picture_id_ = 6;
    picture_y_ = 690;
  }
 protected:
  void getInfo(uint8_t mode) override {
    if (mode == rm_msgs::ShootCmd::STOP) display_info_ = "stop";
    else if (mode == rm_msgs::ShootCmd::READY) display_info_ = "ready";
    else if (mode == rm_msgs::ShootCmd::PUSH) display_info_ = "push";
  }
};

class UiSentry : public UiManual {
 public:
  explicit UiSentry(Referee *referee) : UiManual(referee) {
    display_title_ = "sentry";
    picture_id_ = 10;
    picture_y_ = 590;
  }
 protected:
  void getInfo(uint8_t mode) override {
    if (mode == 0) display_info_ = "standby";
    else display_info_ = "attack";
  }
};

class UiTarget : public UiManual {
 public:
  explicit UiTarget(Referee *referee) : UiManual(referee) {
    display_title_ = "target";
    picture_id_ = 7;
    picture_y_ = 640;
  }
  void display(const ros::Time &time, const std::string &type, const std::string &color, bool base_only) {
    if (operate_type_ != ADD && last_flag_ == base_only && last_type_ == type && last_color_ == color) return;
    getTargetInfo(type, color, base_only);
    if (operate_type_ == ADD) {
      if (!init_flag_) {
        init_time_ = time;
        init_flag_ = true;
      }
      if (time - init_time_ <= ros::Duration(0.2))
        referee_->drawString(picture_id_ + 20, picture_x_, picture_y_, display_title_, WHITE, ADD);
      else referee_->drawString(picture_id_, picture_x_ + 120, picture_y_, display_info_, color_, ADD);
    } else {
      referee_->drawString(picture_id_, picture_x_ + 120, picture_y_, display_info_, color_, operate_type_);
      last_flag_ = base_only;
      last_type_ = type;
      last_color_ = color;
      init_flag_ = false;
    }
  }
 protected:
  void getTargetInfo(const std::string &type, const std::string &color, bool base_only) {
    if (type == "buff") display_info_ = "buff";
    else if (base_only) display_info_ = "base";
    else display_info_ = "armor";
    if (color == "red") color_ = PINK;
    else color_ = CYAN;
  }
  std::string last_type_, last_color_;
};

class UiCover : public UiManual {
 public:
  explicit UiCover(Referee *referee) : UiManual(referee) {
    picture_id_ = 11;
    picture_x_ = 900;
    picture_y_ = 740;
    last_flag_ = true;
  }
  void display(bool close_flag) {
    if (close_flag != last_flag_) {
      if (!close_flag) referee_->drawString(picture_id_, picture_x_, picture_y_, "cover open", GREEN, ADD);
      else referee_->drawString(picture_id_, picture_x_, picture_y_, "cover open", GREEN, DELETE);
    }
    last_flag_ = close_flag;
  }
};

class UiAuto : public UiBase {
 public:
  explicit UiAuto(Referee *referee) : UiBase(referee) {};
  virtual void display(const ros::Time &time) {
    if (operate_type_ != ADD && time - last_update_ < ros::Duration(0.5)) return;
    getInfo();
    referee_->drawString(picture_id_, picture_x_, picture_y_, display_info_, color_, operate_type_);
    last_update_ = time;
  }
  virtual void display(const ros::Time &time, uint8_t mode) {}
 protected:
  virtual void getInfo() {};
  ros::Time last_update_ = ros::Time::now();
};

class UiCapacitor : public UiAuto {
 public:
  explicit UiCapacitor(Referee *referee) : UiAuto(referee) {
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
    sprintf(power_string, "cap: %1.0f%%", cap_power);
    display_info_ = power_string;
  }
};

class UiArmor : public UiAuto {
 public:
  UiArmor(Referee *referee, int armor_id) : UiAuto(referee) { armor_id_ = armor_id; }
  void display(const ros::Time &time) override {
    if (referee_->referee_data_.robot_hurt_.hurt_type_ == 0x00
        && referee_->referee_data_.robot_hurt_.armor_id_ == armor_id_) {
      updateArmorPosition();
      referee_->drawCircle(armor_id_, picture_x_, picture_y_, 50, color_, ADD);
      last_update_ = time;
      delete_flag_ = false;
      referee_->referee_data_.robot_hurt_.hurt_type_ = 0x01;
    }
    if (!delete_flag_ && time - last_update_ > ros::Duration(0.5)) {
      referee_->drawCircle(armor_id_, 0, 0, 0, YELLOW, DELETE);
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

class UiWarning : public UiAuto {
 public:
  explicit UiWarning(Referee *referee) : UiAuto(referee) {
    picture_id_ = 9;
    picture_x_ = 900;
    picture_y_ = 690;
  };
  void display(const ros::Time &time, uint8_t mode) override {
    if (mode != rm_msgs::ChassisCmd::GYRO && time - last_update_ > ros::Duration(2.0)) {
      if (operate_type_ == ADD) operate_type_ = DELETE;
      else operate_type_ = ADD;
      referee_->drawString(picture_id_, picture_x_, picture_y_, "please spin", YELLOW, operate_type_);
      last_update_ = time;
    }
  }
};
} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
