//
// Created by peter on 2020/12/3.
//

#ifndef RM_MANUAL_MANUAL_BASE_H_
#define RM_MANUAL_MANUAL_BASE_H_

#include "rm_manual/common/data.h"
#include "rm_manual/common/input_event.h"
#include "rm_manual/referee/ui.h"

#include <iostream>
#include <queue>
#include <tf/transform_listener.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <controller_manager_msgs/SwitchController.h>

namespace rm_manual {

class ManualBase {
 public:
  explicit ManualBase(ros::NodeHandle &nh);
  enum { PASSIVE, IDLE, RC, PC };
  virtual void run();
 protected:
  void checkReferee(const ros::Time &time);
  void checkSwitch(const ros::Time &time);
  virtual void checkKeyboard();
  virtual void updateRc();
  virtual void updatePc();
  virtual void sendCommand(const ros::Time &time) = 0;
  virtual void drawUi() {};

  // Referee
  virtual void chassisOutputOn() { ROS_INFO("Chassis output ON"); }
  virtual void gimbalOutputOn() { ROS_INFO("Gimbal output ON"); }
  virtual void shooterOutputOn() { ROS_INFO("Shooter output ON"); }

  // Remote Controller
  virtual void remoteControlTurnOff();
  virtual void remoteControlTurnOn();
  virtual void leftSwitchDown() {};
  virtual void leftSwitchMid() {};
  virtual void leftSwitchUp() {};
  virtual void rightSwitchDown() { state_ = IDLE; }
  virtual void rightSwitchMid() { state_ = RC; }
  virtual void rightSwitchUp() { state_ = PC; }

  // Keyboard
  virtual void wPress() {};
  virtual void wRelease() {};
  virtual void sPress() {};
  virtual void sRelease() {};
  virtual void aPress() {};
  virtual void aRelease() {};
  virtual void dPress() {};
  virtual void dRelease() {};
  virtual void mouseLeftPress() {};
  virtual void mouseLeftRelease() {};
  virtual void mouseRightPress() {};
  virtual void mouseRightRelease() {};
  virtual void xPress() {};

  Data data_;
  ros::NodeHandle nh_;
  rm_common::ControllerManager controller_manager_;

  bool remote_is_open_{};
  int state_ = PASSIVE;
  InputEvent switch_right_down_event_, switch_right_mid_event_, switch_right_up_event_, switch_left_down_event_,
      switch_left_mid_event_, switch_left_up_event_, chassis_power_on_, gimbal_power_on_, shooter_power_on_, x_event_,
      w_event_, s_event_, a_event_, d_event_, mouse_left_event_, mouse_right_event_;
};

}
#endif // RM_MANUAL_MANUAL_BASE_H_
