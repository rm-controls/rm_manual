//
// Created by peter on 2020/12/3.
//

#ifndef RM_MANUAL_INCLUDE_RM_MANUAL_MANUAL_COMMON_H_
#define RM_MANUAL_INCLUDE_RM_MANUAL_MANUAL_COMMON_H_

#include "rm_manual/common/data.h"
#include "rm_manual/common/command_sender.h"
#include "rm_manual/common/controller_loader.h"
#include "rm_manual/common/calibration_manager.h"
#include "rm_manual/referee/ui.h"

#include <iostream>
#include <queue>
#include <tf/transform_listener.h>
#include <controller_manager_msgs/SwitchController.h>

#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>

namespace rm_manual {

class ManualBase {
 public:
  explicit ManualBase(ros::NodeHandle &nh);
  ~ManualBase() {
    delete controller_loader_;
    delete calibration_manager_;
  }
  enum { PASSIVE, IDLE, RC, PC };
  void run();
 protected:
  void checkSwitch(const ros::Time &time);
  void checkKeyboard(const ros::Time &time);
  virtual void setZero() = 0;
  virtual void sendCommand(const ros::Time &time) = 0;
  virtual void drawUi() {};

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
  }
  virtual void leftSwitchDown() {};
  virtual void leftSwitchMid() {};
  virtual void leftSwitchUp() {};
  virtual void rightSwitchDown() { state_ = IDLE; }
  virtual void rightSwitchMid() { state_ = RC; }
  virtual void rightSwitchUp() { state_ = PC; }

  // Keyboard
  virtual void qPress() {};
  virtual void wPress() {};
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

  // Press in same time
  virtual void mouseLeftRightPress() {};
  virtual void ctrlZPress() {};
  virtual void ctrlWPress() {};

  Data data_;
  Ui *ui_;
  ControllerLoader *controller_loader_;
  CalibrationManager *calibration_manager_;
  SwitchControllersService *switch_state_ctrl_srv_, *switch_base_ctrl_srv_{};

  bool remote_is_open_{};
  ros::NodeHandle nh_;
  int state_ = PASSIVE;
  ros::Time last_release_q_, last_release_w_, last_release_e_, last_release_r_, last_release_t_, last_release_a_,
      last_release_s_, last_release_d_, last_release_f_, last_release_g_, last_release_z_, last_release_x_,
      last_release_c_, last_release_v_, last_release_b_, last_release_shift_, last_release_mouse_left_,
      last_release_mouse_right_, last_release_mouse_right_left_, last_release_ctrl_z_, last_release_ctrl_w_;

};

}
#endif // RM_MANUAL_MANUAL_COMMON_H_
