//
// Created by peter on 2020/12/3.
//

#ifndef RM_MANUAL_INCLUDE_RM_MANUAL_MANUAL_COMMON_H_
#define RM_MANUAL_INCLUDE_RM_MANUAL_MANUAL_COMMON_H_

#include <iostream>
#include <queue>
#include <utility>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <controller_manager_msgs/SwitchController.h>

#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include "data.h"
#include "controller_manager.h"
#include "command_sender.h"

namespace rm_manual {

class Manual {
 public:
  explicit Manual(ros::NodeHandle &nh);
  void run();
 protected:
  void checkSwitch(const ros::Time &time);
  void checkKeyboard(const ros::Time &time);
  virtual void leftSwitchDown() {};
  virtual void leftSwitchMid() {};
  virtual void leftSwitchUp() {};
  virtual void rightSwitchDown() {};
  virtual void rightSwitchMid() {};
  virtual void rightSwitchUp() {};
  virtual void ctrlQPress() {};
  virtual void ctrlWPress() {};
  virtual void qPress(ros::Duration period) {};
  virtual void wPress(ros::Duration period) {};
  virtual void ePress(ros::Duration period) {};
  virtual void rPress(ros::Duration period) {};
  virtual void aPress(ros::Duration period) {};
  virtual void sPress(ros::Duration period) {};
  virtual void dPress(ros::Duration period) {};
  virtual void fPress(ros::Duration period) {};
  virtual void gPress(ros::Duration period) {};
  virtual void zPress(ros::Duration period) {};
  virtual void xPress(ros::Duration period) {};
  virtual void cPress(ros::Duration period) {};
  virtual void vPress(ros::Duration period) {};
  virtual void bPress(ros::Duration period) {};
  virtual void shiftPress(ros::Duration period) {};
  virtual void mouseLeftPress(ros::Duration period) {};
  virtual void mouseRightPress(ros::Duration period) {};
  virtual void mouseLeftRightPress(ros::Duration period) {};
  virtual void remoteControlTurnOff() {};
  virtual void remoteControlTurnOn() {};

  Data data_;
  ros::NodeHandle nh_;
  ControllerManager *controller_manager_;
  bool is_rc_opened_;

  ros::Time last_press_q_, last_press_w_, last_press_e_, last_press_r_, last_press_t_, last_press_a_, last_press_s_,
      last_press_d_, last_press_f_, last_press_g_, last_press_z_, last_press_x_, last_press_c_, last_press_v_,
      last_press_b_, last_press_shift_, last_press_mouse_left_, last_press_mouse_right_, last_press_mouse_right_left_;
};

}
#endif //RM_MANUAL_INCLUDE_RM_MANUAL_MANUAL_COMMON_H_
