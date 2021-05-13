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
  int performance_system_; // Performance level system
};

class PowerManagerData {
 public:
  PowerManagerData() = default;
  ~PowerManagerData() = default;
  float parameters[4] = {0};
  void read(const std::vector<uint8_t> &rx_buffer);

 private:
  void DTP_Received_CallBack(unsigned char Receive_Byte);
  void Receive_CallBack(unsigned char PID, unsigned char Data[8]);

  unsigned char Receive_Buffer[1024] = {0};
  unsigned char PingPong_Buffer[1024] = {0};
  unsigned int Receive_BufCounter = 0;
  float Int16ToFloat(unsigned short data0);
};

class Referee {
 public:
  Referee() = default;
  ~Referee() = default;
  void init(ros::NodeHandle nh);
  void read();
  void run();
  void drawCircle(int center_x, int center_y, int radius, int picture_name,
                  GraphicColorType color, uint8_t operate_type);
  void drawString(int x, int y, int picture_name, GraphicColorType color, uint8_t operate_type, std::string data);
  void sendInteractiveData(int data_cmd_id, int receiver_id, const std::vector<uint8_t> &data);

  double getActualBulletSpeed(int shoot_speed) const;
  double getUltimateBulletSpeed(int shoot_speed) const;

  ros::Time last_press_time_g_ = ros::Time::now();
  ros::Time last_press_time_r_ = ros::Time::now();
  ros::Time last_press_time_f_ = ros::Time::now();
  ros::Time last_press_time_q_ = ros::Time::now();
  ros::Time last_press_time_c_ = ros::Time::now();

  bool gyro_flag_ = false;
  bool twist_flag_ = false;
  bool burst_flag_ = false;
  bool only_attack_base_flag_ = false;

  bool is_chassis_passive_ = true;
  bool is_gimbal_passive_ = true;
  bool is_shooter_passive_ = true;
  bool is_open_ = false;
  int robot_id_ = 0;
  int client_id_ = 0;

  bool chassis_update_flag_ = true;
  bool gimbal_update_flag_ = true;
  bool shooter_update_flag_ = true;
  bool attack_mode_update_flag_ = true;

  ros::NodeHandle nh_;
  rm_msgs::DbusData dbus_data_;
  RefereeData referee_data_{};
  PowerManagerData power_manager_data_;
  ros::Subscriber dbus_sub_;

  ros::Publisher referee_pub_;
  ros::Publisher power_manager_pub_;

  ros::Time last_send_ = ros::Time::now();

  ros::Time last_hurt_id0_ = ros::Time::now();
  ros::Time last_hurt_id1_ = ros::Time::now();
  ros::Time last_hurt_id2_ = ros::Time::now();
  ros::Time last_hurt_id3_ = ros::Time::now();

  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;

  rm_msgs::Referee referee_pub_data_;
  rm_msgs::PowerManagerData power_manager_pub_data_;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) {
    dbus_data_ = *data;
  }
 private:
  void getId();
  void unpack(const std::vector<uint8_t> &rx_buffer);
  void getData(uint8_t *frame);

  serial::Serial serial_;
  std::vector<uint8_t> rx_data_;
  UnpackData referee_unpack_obj{};

  const std::string serial_port_ = "/dev/usbReferee";
  const int kUnpackLength = 256;
  const int kProtocolFrameLength = 128, kProtocolHeaderLength = 5, kProtocolCmdIdLength = 2, kProtocolTailLength = 2;
};

uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8);
uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length);
void appendCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t getCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
void appendCRC16CheckSum(unsigned char *pchMessage, unsigned int dwLength);

#endif //SRC_RM_BRIDGE_INCLUDE_RT_RT_REFEREE_H_