//
// Created by peter on 2020/12/3.
//

#ifndef RM_MANUAL_MANUAL_BASE_H_
#define RM_MANUAL_MANUAL_BASE_H_

#include <iostream>
#include <queue>
#include <tf/transform_listener.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_loader.h>
#include <rm_common/decision/calibration_manager.h>
#include <controller_manager_msgs/SwitchController.h>

#include "rm_manual/common/data.h"
#include "rm_manual/common/input_event.h"
#include "rm_manual/referee/ui.h"

namespace rm_manual {

class ManualBase {
 public:
  explicit ManualBase(ros::NodeHandle &nh);
  ~ManualBase() {
    delete controller_loader_;
    delete calibration_manager_;
  }
  enum { PASSIVE, IDLE, RC, PC };
  virtual void run();
 protected:
  void checkReferee(const ros::Time &time);
  void checkSwitch(const ros::Time &time);
  void checkKeyboard();
  virtual void updateRc();
  virtual void updatePc();
  virtual void sendCommand(const ros::Time &time) = 0;
  virtual void drawUi() {};

  // Referee
  virtual void chassisOutputOn() {};
  virtual void gimbalOutputOn() {};
  virtual void shooterOutputOn() {};

  // Remote Controller
  virtual void remoteControlTurnOff() {
    switch_base_ctrl_srv_->flipControllers();
    switch_base_ctrl_srv_->callService();
    state_ = PASSIVE;
  }
  virtual void remoteControlTurnOn() {
    switch_base_ctrl_srv_->switchControllers();
    switch_base_ctrl_srv_->callService();
    state_ = IDLE;
    calibration_manager_->reset();
  }
  virtual void leftSwitchDown() {};
  virtual void leftSwitchMid() {};
  virtual void leftSwitchUp() {};
  virtual void rightSwitchDown() { state_ = IDLE; }
  virtual void rightSwitchMid() { state_ = RC; }
  virtual void rightSwitchUp() { state_ = PC; }

  // Keyboard
  virtual void qPress() {};
  virtual void wPress(ros::Duration duration) {};
  virtual void ePress() {};
  virtual void rPress() {};
  virtual void aPress() {};
  virtual void sPress() {};
  virtual void dPress() {};
  virtual void fPress() {};
  virtual void gPress() {};
  virtual void zPress() {};
  virtual void xPress() {};
  virtual void cPress() {};
  virtual void vPress() {};
  virtual void bPress() {};
  virtual void shiftPress() {};
  virtual void mouseLeftPress() {};
  virtual void mouseRightPress() {};
  virtual void ctrlRPress() {};
  virtual void ctrlVPress() {};
  virtual void ctrlCPress() {};
  virtual void ctrlZPress() {};

  virtual void qRelease() {};
  virtual void wRelease() {};
  virtual void eRelease() {};
  virtual void rRelease() {};
  virtual void aRelease() {};
  virtual void sRelease() {};
  virtual void dRelease() {};
  virtual void fRelease() {};
  virtual void gRelease() {};
  virtual void zRelease() {};
  virtual void xRelease() {};
  virtual void cRelease() {};
  virtual void vRelease() {};
  virtual void bRelease() {};
  virtual void shiftRelease() {};
  virtual void mouseLeftRelease() {};
  virtual void mouseRightRelease() {};
  virtual void ctrlRRelease() {};
  virtual void ctrlVRelease() {};
  virtual void ctrlCRelease() {};
  virtual void ctrlZRelease() {};

  Data data_;
  rm_common::ControllerLoader *controller_loader_;
  rm_common::CalibrationManager *calibration_manager_;
  rm_common::SwitchControllersService *switch_state_ctrl_srv_{}, *switch_base_ctrl_srv_{};

  bool remote_is_open_{};
  ros::NodeHandle nh_;
  int state_ = PASSIVE;
  RisingInputEvent w_press_event_;
};

}
#endif // RM_MANUAL_MANUAL_BASE_H_
