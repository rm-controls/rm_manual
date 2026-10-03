//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include "rm_manual/chassis_gimbal_manual.h"
#include <rm_common/decision/calibration_queue.h>
#include <std_srvs/Empty.h>
#include <angles/angles.h>
#include <unordered_set>

namespace rm_manual
{
class ChassisGimbalShooterManual : public ChassisGimbalManual
{
public:
  ChassisGimbalShooterManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

protected:
  void ecatReconnected() override;
  void checkReferee() override;
  void checkWheelsOnline();
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void sendCommand(const ros::Time& time) override;
  void updateWheelsState(const rm_ecat_msgs::RmEcatStandardSlaveReadings::ConstPtr& data,
                         const std::vector<std::string>& chassis_motor);
  void wheelsOnlineCallback(const rm_ecat_msgs::RmEcatStandardSlaveReadings::ConstPtr& data);
  void chassisOutputOn() override;
  void shooterOutputOn() override;
  void gimbalOutputOn() override;
  void selfInspectionStart()
  {
    shooter_calibration_->reset();
  };
  void gameStart()
  {
    shooter_calibration_->reset();
  };
  void remoteControlTurnOff() override;
  void remoteControlTurnOn() override;
  void robotDie() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchDownRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchMidOn(ros::Duration duration);
  void leftSwitchUpRise() override;
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data) override;
  void shootBeforehandCmdCallback(const rm_msgs::ShootBeforehandCmd ::ConstPtr& data) override;
  void suggestFireCallback(const std_msgs::Bool::ConstPtr& data) override;
  void trackCallback(const rm_msgs::TrackData::ConstPtr& data) override;
  void shootDataCallback(const rm_msgs::ShootData::ConstPtr& data) override;
  void ballisticSolutionCallback(const std_msgs::Float32MultiArray::ConstPtr& data) override;
  void protectStateCallback(const std_msgs::Bool::ConstPtr& data) override;
  void leftSwitchUpOn(ros::Duration duration);
  void leftSwitchUpFall();
  void mouseLeftPress();
  void mouseLeftRelease()
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    prepare_shoot_ = true;
  }
  virtual void mouseRightPress();
  void mouseRightRelease()
  {
    if (deployed_)
      return;
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  }
  void mouseRightRising();
  void wPress() override;
  void aPress() override;
  void sPress() override;
  void dPress() override;
  void wPressing() override;
  void aPressing() override;
  void sPressing() override;
  void dPressing() override;
  void wRelease() override;
  void aRelease() override;
  void sRelease() override;
  void dRelease() override;
  virtual void gPress();
  virtual void zPress();
  virtual void vPress();
  virtual void xPress();
  virtual void ePress();
  virtual void eRelease();
  virtual void cPress();
  virtual void bPress();
  virtual void bRelease();
  virtual void xRelease();
  virtual void shiftPress();
  virtual void shiftRelease();
  virtual void rPress();
  virtual void qPress();

  void ctrlFPress()
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void ctrlVPress();
  void ctrlRPress();
  void ctrlZPress();
  void ctrlXPress();
  virtual void ctrlCPress();
  virtual void ctrlQPress();
  virtual void ctrlBPress();

  InputEvent self_inspection_event_, game_start_event_, e_event_, c_event_, g_event_, q_event_, b_event_, x_event_,
      r_event_, v_event_, z_event_, ctrl_f_event_, ctrl_v_event_, ctrl_b_event_, ctrl_q_event_, ctrl_r_event_,
      ctrl_z_event_, ctrl_c_event_, ctrl_x_event_, shift_event_, mouse_left_event_, mouse_right_event_;
  rm_common::ShooterCommandSender* shooter_cmd_sender_{};
  rm_common::CameraSwitchCommandSender* camera_switch_cmd_sender_{};
  rm_common::JointPositionBinaryCommandSender* scope_cmd_sender_{};
  rm_common::JointPositionBinaryCommandSender* image_transmission_cmd_sender_{};
  rm_common::ChassisActiveSuspensionCommandSender* chassis_active_sus_cmd_sender_{};
  rm_common::BallisticSolverRequestCommandSender* ballistic_solver_request_cmd_sender_{};

  rm_common::SwitchDetectionCaller* switch_detection_srv_{};
  rm_common::SwitchDetectionCaller* switch_detection_left_srv_{};
  rm_common::SwitchDetectionCaller* switch_armor_target_srv_{};
  rm_common::ServiceCallerBase<std_srvs::Empty>* relocate_srv_{};
  rm_common::ColorChangeServiceCaller* color_change_srv_{};
  rm_common::TrackerResetServiceCaller* tracker_reset_srv_{};

  rm_common::CalibrationQueue* chassis_calibration_;
  rm_common::CalibrationQueue* shooter_calibration_;
  rm_common::CalibrationQueue* gimbal_calibration_;

  ros::Subscriber wheel_online_sub_;
  ros::Time last_wheels_power_time_;
  std::vector<std::string> chassis_motor_;
  std::vector<bool> wheels_online_state_;

  std_msgs::Float32MultiArray ballistic_solution_;
  ros::Time last_ballistic_solution_request_time_;

  bool prepare_shoot_{ false }, is_balance_{ false }, use_scope_{ false }, deployed_{ false },
      is_follow_yaw_reverse_{ false }, all_wheel_offline_{ false }, protect_state_{ false };
  double ballistic_yaw_{}, ballistic_pitch_{};
  double ballistic_yaw_step_{}, ballistic_pitch_step_{};
  double deploy_pitch_{}, deploy_yaw_;
  uint8_t last_shoot_freq_{};
  double scale_{};
};
}  // namespace rm_manual
