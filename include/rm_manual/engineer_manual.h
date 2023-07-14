//
// Created by qiayuan on 5/23/21.
//

#pragma once

#include "rm_manual/chassis_gimbal_manual.h"

#include <utility>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <rm_common/decision/calibration_queue.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_msgs/EngineerAction.h>
#include <rm_msgs/MultiDofCmd.h>
#include <rm_msgs/GpioData.h>
#include <angles/angles.h>

namespace rm_manual
{
class EngineerManual : public ChassisGimbalManual
{
public:
  enum ControlMode
  {
    MANUAL,
    MIDDLEWARE
  };

  enum JointMode
  {
    SERVO,
    JOINT
  };

  enum GimbalMode
  {
    RATE,
    DIRECT
  };

  enum SpeedMode
  {
    LOW,
    NORMAL,
    FAST,
    EXCHANGE
  };

  enum ExchangeProcess
  {
    YZ,
    ROLL_YAW,
    XYZ,
    PITCH,
    PUSH,
    FINISH,
  };

  struct JointInfo
  {
    double offset;
    double max_position;
    double min_position;
    double current_position;
    bool judgeJointPosition()
    {
      return (abs(current_position - offset - min_position) <= 0.01 ||
              abs(max_position + offset - current_position) <= 0.01) ?
                 true :
                 false;
    }
  };

  struct ExchangeInfo
  {
    int process{ YZ };
    ros::Time process_start_time{};
    bool finish_exchange{ false }, recorded_time{ false };
    std_msgs::Bool enter_auto_exchange{};
    double single_process_max_time{}, link7_length{};
    std::vector<double> xyz_offset{ 0, 0, 0 };
    std::vector<double> servo_scales{ 0, 0, 0, 0, 0, 0 };
    std::vector<double> servo_p{ 0, 0, 0, 0, 0, 0 };
    std::vector<double> servo_errors{ 0, 0, 0, 0, 0, 0 };
    std::vector<double> servo_error_tolerance{ 0, 0, 0, 0, 0, 0 };
    void initComputerValue()
    {
      for (int i = 0; i < (int)servo_scales.size(); ++i)
      {
        servo_errors[i] = 0;
        servo_scales[i] = 0;
      }
    }
    void nextProcess()
    {
      if (process != FINISH)
      {
        process++;
        process_start_time = ros::Time::now();
        recorded_time = false;
      }
      else
      {
        finish_exchange = true;
        recorded_time = false;
        process_start_time = ros::Time::now();
        ROS_INFO_STREAM("FINISH");
      }
    }
    bool checkTimeOut()
    {
      ROS_INFO_STREAM((ros::Time::now() - process_start_time).toSec());
      if (!recorded_time)
      {
        process_start_time = ros::Time::now();
        recorded_time = true;
      }
      return ((ros::Time::now() - process_start_time).toSec() >= single_process_max_time) ? true : false;
    }
    void quitExchange()
    {
      process = YZ;
      recorded_time = false;
      enter_auto_exchange.data = false;
      finish_exchange = false;
    }
    void printProcess()
    {
      switch (process)
      {
        case YZ:
          ROS_INFO_STREAM("YZ");
          break;
        case ROLL_YAW:
          ROS_INFO_STREAM("ROLL_YAW");
          break;
        case XYZ:
          ROS_INFO_STREAM("XYZ");
          break;
        case PITCH:
          ROS_INFO_STREAM("PITCH");
          break;
        case PUSH:
          ROS_INFO_STREAM("PUSH");
          break;
      }
    }
  };

  EngineerManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

private:
  void changeSpeedMode(SpeedMode speed_mode);
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void sendCommand(const ros::Time& time) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void actionFeedbackCallback(const rm_msgs::EngineerFeedbackConstPtr& feedback);
  void actionDoneCallback(const actionlib::SimpleClientGoalState& state, const rm_msgs::EngineerResultConstPtr& result);
  void runStepQueue(const std::string& step_queue_name);
  void initMode();
  void actionActiveCallback()
  {
    operating_mode_ = MIDDLEWARE;
  }
  void remoteControlTurnOff() override;
  void chassisOutputOn() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchUpRise() override;
  void leftSwitchUpFall();
  void leftSwitchDownFall();
  void leftSwitchDownRise() override;
  void ctrlQPress();
  void ctrlWPress();
  void ctrlEPress();
  void ctrlAPress();
  void ctrlSPress();
  void ctrlZPress();
  void ctrlZPressing();
  void ctrlZRelease();
  void ctrlXPress();
  void ctrlCPress();
  void ctrlDPress();
  void ctrlVRelease();
  void ctrlVPress();
  void ctrlBPress();
  void ctrlFPress();
  void ctrlGPress();
  void ctrlRPress();
  void shiftPressing();
  void shiftRelease();
  void shiftZPress();
  void shiftXPress();
  void shiftCPress();
  void shiftBPress();
  void shiftBRelease();
  void shiftGPress();
  void shiftRPressing();
  void shiftRRelease();
  void shiftVPress();
  void shiftVRelease();
  void shiftFPress();
  void rPress();
  void qPressing();
  void qRelease();
  void ePressing();
  void eRelease();
  void zPressing();
  void zRelease();
  void xPress();
  void cPressing();
  void cRelease();
  void bPressing();
  void bRelease();
  void vPressing();
  void vRelease();
  void fPress();
  void fRelease();
  void gPress();
  void gRelease();

  void mouseLeftRelease();
  void mouseRightRelease();
  void gpioStateCallback(const rm_msgs::GpioData::ConstPtr& data);
  void stoneNumCallback(const std_msgs::String ::ConstPtr& data);

  void updateServo(const rm_msgs::DbusData::ConstPtr& dbus_data);
  void servoAutoExchange();
  void enterAutoExchange();
  void quitAutoExchange();
  void computeServoError();
  void computeServoScale();
  void manageExchangeProcess();

  int checkJointsLimit();

  bool reversal_motion_{}, change_flag_{};
  int operating_mode_{}, servo_mode_{}, gimbal_mode_{}, stone_num_{};
  double angular_z_scale_{};
  double fast_speed_scale_{}, normal_speed_scale_{}, low_speed_scale_{}, exchange_speed_scale_{};
  double gyro_scale_{}, fast_gyro_scale_{}, normal_gyro_scale_{}, low_gyro_scale_{}, exchange_gyro_scale_{};
  std::string prefix_{}, root_{}, drag_state_{ "on" }, max_temperature_joint_{}, joint_temperature_{},
      reversal_state_{}, gripper_state_{};
  JointInfo joint1_{}, joint2_{}, joint3_{};
  std::vector<JointInfo> joints_{};
  ExchangeInfo exchange_info_{};

  ros::Publisher exchanger_update_pub_;
  ros::Subscriber gripper_state_sub_, stone_num_sub_;
  actionlib::SimpleActionClient<rm_msgs::EngineerAction> action_client_;

  rm_msgs::GpioData gpio_state_;
  rm_common::Vel3DCommandSender* servo_command_sender_;
  rm_common::MultiDofCommandSender* reversal_command_sender_;
  rm_common::ServiceCallerBase<std_srvs::Empty>* servo_reset_caller_;
  rm_common::JointPositionBinaryCommandSender *drag_command_sender_, *joint7_command_sender_;
  rm_common::CalibrationQueue* calibration_gather_{};
  InputEvent left_switch_up_event_, left_switch_down_event_, ctrl_q_event_, ctrl_a_event_, ctrl_z_event_, ctrl_w_event_,
      ctrl_s_event_, ctrl_x_event_, ctrl_e_event_, ctrl_d_event_, ctrl_c_event_, ctrl_b_event_, ctrl_v_event_, z_event_,
      q_event_, e_event_, x_event_, c_event_, v_event_, b_event_, f_event_, shift_z_event_, shift_x_event_,
      shift_c_event_, shift_v_event_, shift_b_event_, shift_g_event_, ctrl_r_event_, shift_q_event_, shift_f_event_,
      shift_e_event_, ctrl_g_event_, shift_r_event_, ctrl_f_event_, shift_event_, g_event_, r_event_, mouse_left_event_,
      mouse_right_event_;
};
}  // namespace rm_manual
