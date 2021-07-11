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
  UiBase(Referee *referee) : referee_(referee) {}
  void setOperateType(GraphicOperateType operate_type) { operate_type_ = operate_type; }
 protected:
  Referee *referee_;
  GraphicOperateType operate_type_ = ADD;
  GraphicColorType color_ = YELLOW;
  std::string display_info_;
  int picture_id_, picture_x_, picture_y_;
};

class UiControllers : public UiBase {
 public:
  UiControllers(Referee *referee) : UiBase(referee) {};
  void display(uint8_t mode, bool flag = false) {
    if (operate_type_ != ADD && last_mode_ == mode && last_flag_ == flag) return;
    color_ = flag ? ORANGE : YELLOW;
    getInfo(mode);
    referee_->drawString(picture_x_, picture_y_, picture_id_, display_info_, color_, operate_type_);
    ROS_INFO("display");
    last_mode_ = mode;
    last_flag_ = flag;
  }
 protected:
  virtual void getInfo(uint8_t mode) {};
  uint8_t last_mode_;
  bool last_flag_ = false;
};

class UiChassis : public UiControllers {
 public:
  UiChassis(Referee *referee) : UiControllers(referee) {
    last_mode_ = rm_msgs::ChassisCmd::FOLLOW;
    picture_id_ = 0;
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

class UiGimbal : public UiControllers {
 public:
  UiGimbal(Referee *referee) : UiControllers(referee) {
    last_mode_ = rm_msgs::GimbalCmd::RATE;
    picture_id_ = 1;
    picture_x_ = 1470;
    picture_y_ = 740;
  }
 protected:
  void getInfo(uint8_t mode) override {
    if (mode == rm_msgs::GimbalCmd::RATE) display_info_ = "gimbal:rate";
    else if (mode == rm_msgs::GimbalCmd::TRACK) display_info_ = "gimbal:track";
  }
};

class UiShooter : public UiControllers {
 public:
  UiShooter(Referee *referee) : UiControllers(referee) {
    last_mode_ = rm_msgs::ShootCmd::STOP;
    picture_id_ = 2;
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
} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
