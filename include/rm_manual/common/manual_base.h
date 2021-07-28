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
  void checkSwitch(const ros::Time &time);
  virtual void checkReferee();
  virtual void checkKeyboard() {};
  virtual void updateRc();
  virtual void updatePc();
  virtual void sendCommand(const ros::Time &time) = 0;
  virtual void drawUi(const ros::Time &time) { data_.referee_.sendUi(time); }

  // Referee
  virtual void chassisOutputOn() { ROS_INFO("Chassis output ON"); }
  virtual void gimbalOutputOn() { ROS_INFO("Gimbal output ON"); }
  virtual void shooterOutputOn() { ROS_INFO("Shooter output ON"); }
  virtual void robotDie();
  virtual void robotRevive();

  // Remote Controller
  virtual void remoteControlTurnOff();
  virtual void remoteControlTurnOn();
  virtual void leftSwitchDownRise() {};
  virtual void leftSwitchMidRise() {};
  virtual void leftSwitchUpRise() {};
  virtual void rightSwitchDownRise() { state_ = IDLE; }
  virtual void rightSwitchMidRise() { state_ = RC; }
  virtual void rightSwitchUpRise() { state_ = PC; }

  Data data_;
  ros::NodeHandle nh_;
  rm_common::ControllerManager controller_manager_;

  bool remote_is_open_{};
  int state_ = PASSIVE;
  InputEvent robot_hp_event_, right_switch_down_event_, right_switch_mid_event_, right_switch_up_event_,
      left_switch_down_event_, left_switch_mid_event_, left_switch_up_event_;
};

}
#endif // RM_MANUAL_MANUAL_BASE_H_
