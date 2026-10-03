//
// Created by longyu on 9/21/25.
//

#pragma once

#include "rm_manual/balance_manual.h"
#include <rm_msgs/LeggedChassisMode.h>
#include <rm_msgs/LeggedUpstairStatus.h>
#include <rm_ecat_msgs/RmEcatStandardSlaveReadings.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

namespace rm_manual
{
class SeriesLeggedManual : public BalanceManual
{
public:
  SeriesLeggedManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void shiftPress() override;
  void shiftRelease() override;
  void bPress() override;
  void bPressing();
  void bRelease() override;
  void ctrlZPress() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rPress() override;
  void rPressing();
  void rRelease();
  void ctrlXPress() override;
  void ctrlWPressing();
  void zPress() override;
  void robotDie() override;
  void robotRevive() override;
  void ePress() override;
  void eRelease() override;
  void qPress() override;
  void vPress() override;
  void vRelease();
  void fPress();
  void fRelease();

  void aPress() override;
  void aPressing() override;
  void aRelease() override;
  void dPress() override;
  void dPressing() override;
  void dRelease() override;

  void leftSwitchUpRise() override;

  void leftSwitchMidRise() override;

  void leftSwitchDownRise() override;

  void rightSwitchDownOn() override;

  void sendCommand(const ros::Time& time) override;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void ctrlGPress();
  void ctrlGRelease();
  void enterSitDown();
  void exitSitDown();
  void xPress() override;
  void reviveMotorOnlineCallback(const rm_ecat_msgs::RmEcatStandardSlaveReadingsConstPtr& msg);
  rm_common::LegCommandSender* legCommandSender_{};

private:
  enum Leg_len_status
  {
    SHORT,
    MID,
    HIGH,
    LEG_RETRACTION,
  };
  ros::Time xPress_time_{}, ctrlXPress_time_{};
  double jump_up_range_threshold_{ 0.8 };
  bool stretch_ = false, stretching_ = false, is_increasing_length_ = false;
  bool is_die_flag_{ false }, sit_down_flag_{ false };
  Leg_len_status leg_len_status_{ SHORT };
  int upstair_leg_len_fsm_{ 0 };
  std::map<Leg_len_status, double> leg_len_map_;
  double target_leg_length_{ 0.20 }, current_leg_length_{};
  ros::Time last_upstairs_time_{};
  double total_tof_len_{}, left_tof_len_{}, right_tof_len{};
  bool left_wheel_online_, right_wheel_online_;
  bool debug_gimbal_flag_{ false };
  bool jump_up_flag_{ false }, down_5cm_stair_flag_{ false };
  InputEvent ctrl_event_, ctrl_g_event_, ctrl_w_event_, f_event_;
  InputEvent revive_motor_online_check_event_;
  ros::Subscriber revive_motor_online_sub_;
  ros::Subscriber left_tof_sensor_sub_, right_tof_sensor_sub_;
  ros::Subscriber unstick_sub_, leg_len_status_sub_, legged_chassis_mode_sub_;
  ros::Publisher recovery_leg_spd_turnback_pub_;
  ros::ServiceClient down_5cm_stair_client_;
  void unstickCallback(const std_msgs::BoolConstPtr& msg);
  void upstairStatusCallback(const rm_msgs::LeggedUpstairStatusConstPtr& msg);
  void leggedChassisModeCallback(const rm_msgs::LeggedChassisModeConstPtr& msg);
  void tofSensorMsgCallback(const sensor_msgs::RangeConstPtr& msg);
  inline void setLegLenStatus(Leg_len_status len_status);
};
}  // namespace rm_manual
