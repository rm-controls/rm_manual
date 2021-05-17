//
// Created by luohx on 20-2-19.
//
#ifndef SRC_RM_BRIDGE_INCLUDE_RT_RT_REFEREE_H_
#define SRC_RM_BRIDGE_INCLUDE_RT_RT_REFEREE_H_

#include <cstdint>
#include <serial/serial.h>
#include <tf/transform_listener.h>

#include <rm_common/ori_tool.h>
#include <rm_msgs/Referee.h>
#include <rm_msgs/PowerManagerData.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <geometry_msgs/Twist.h>

#include "rm_manual/protocol.h"

struct RefereeData {
  GameStatus game_status_;
  GameResult game_result_;
  GameRobotHp game_robot_hp_;
  DartStatus dart_status_;
  IcraBuffDebuffZoneStatus icra_buff_debuff_zone_status;
  EventData event_data_;
  SupplyProjectileAction supply_projectile_action_;
  RefereeWarning referee_warning_;
  DartRemainingTime dart_remaining_time_;
  GameRobotStatus game_robot_status_;
  PowerHeatData power_heat_data_;
  GameRobotPos game_robot_pos_;
  Buff buff_;
  AerialRobotEnergy aerial_robot_energy_;
  RobotHurt robot_hurt_;
  ShootData shoot_data_;
  BulletRemaining bullet_remaining_;
  RfidStatus rfid_status_;
  DartClientCmd dart_client_cmd_;
  StudentInteractiveData student_interactive_data_;
  GraphicDataStruct graphic_data_struct_;
  RobotInteractiveData robot_interactive_data_;
  RobotCommand robot_command_;
};

class PowerManagerData {
 public:
  void read(const std::vector<uint8_t> &rx_buffer);

  float parameters[4] = {0};

  ros::Time last_get_powermanager_data_ = ros::Time::now();
 private:
  void DTP_Received_CallBack(unsigned char Receive_Byte);
  void Receive_CallBack(unsigned char PID, unsigned char Data[8]);
  static float Int16ToFloat(unsigned short data0);

  unsigned char Receive_Buffer[1024] = {0};
  unsigned char PingPong_Buffer[1024] = {0};
  unsigned int Receive_BufCounter = 0;
};

class Referee {
 public:
  void init(ros::NodeHandle nh);
  void read();
  void run();

  double getActualBulletSpeed(int shoot_speed) const;
  double getUltimateBulletSpeed(int shoot_speed) const;

  ros::NodeHandle nh_;
  ros::Publisher referee_pub_;
  ros::Publisher power_manager_pub_;

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_{};

  rm_msgs::Referee referee_pub_data_;
  rm_msgs::PowerManagerData power_manager_pub_data_;
  RefereeData referee_data_{};
  PowerManagerData power_manager_data_;

  bool is_open_ = false;

  int robot_id_ = 0;
  int client_id_ = 0;

 private:
  void unpack(const std::vector<uint8_t> &rx_buffer);
  void getData(uint8_t *frame);
  void getRobotId();
  void publishData();

  void checkForDelay(float key_delay, float cap_data_delay, const ros::Time &now);
  double getArmorPosition();
  void displayArmorInfo(double yaw, const ros::Time &now);
  void displayCapInfo(uint8_t graph_operate_type);
  void displayChassisInfo(uint8_t graph_operate_type);
  void displayGimbalInfo(uint8_t graph_operate_type);
  void displayShooterInfo(uint8_t graph_operate_type);
  void displayAttackTargetInfo(uint8_t graph_operate_type);

  void drawCircle(int center_x, int center_y, int radius, int picture_id,
                  GraphicColorType color, uint8_t operate_type);
  void drawString(int x, int y, int picture_id, std::string data, GraphicColorType color, uint8_t operate_type);
  void sendInteractiveData(int data_cmd_id, int receiver_id, const std::vector<uint8_t> &data);

  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) {
    dbus_data_ = *data;
  }

  serial::Serial serial_;
  const std::string serial_port_ = "/dev/usbReferee";
  const int kUnpackLength = 256;
  const int kProtocolFrameLength = 128, kProtocolHeaderLength = 5, kProtocolCmdIdLength = 2, kProtocolTailLength = 2;
  UnpackObject referee_unpack_obj{};
  std::vector<uint8_t> rx_data_;

  ros::Subscriber dbus_sub_;
  rm_msgs::DbusData dbus_data_;

  ros::Time last_get_referee_data_ = ros::Time::now();

  ros::Time last_press_g_ = ros::Time::now();
  ros::Time last_press_r_ = ros::Time::now();
  ros::Time last_press_q_ = ros::Time::now();
  ros::Time last_press_c_ = ros::Time::now();

  ros::Time last_update_cap_ = ros::Time::now();

  ros::Time last_hurt_armor0_ = ros::Time::now();
  ros::Time last_hurt_armor1_ = ros::Time::now();
  ros::Time last_hurt_armor2_ = ros::Time::now();
  ros::Time last_hurt_armor3_ = ros::Time::now();

  std::string chassis_mode_ = "passive";
  std::string gimbal_mode_ = "passive";
  std::string shooter_mode_ = "passive";

  bool gyro_flag_ = false;
  bool twist_flag_ = false;
  bool burst_flag_ = false;
  bool only_attack_base_flag_ = false;

  bool last_key_f_ = false;

  bool chassis_update_flag_ = true;
  bool gimbal_update_flag_ = true;
  bool shooter_update_flag_ = true;
  bool attack_mode_update_flag_ = true;
  bool cap_update_flag_ = true;

  bool armor0_update_flag_ = false;
  bool armor1_update_flag_ = false;
  bool armor2_update_flag_ = false;
  bool armor3_update_flag_ = false;
};

// CRC verification
uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8);
uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length);
void appendCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t getCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
void appendCRC16CheckSum(unsigned char *pchMessage, unsigned int dwLength);

#endif //SRC_RM_BRIDGE_INCLUDE_RT_RT_REFEREE_H_