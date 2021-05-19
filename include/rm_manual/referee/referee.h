//
// Created by peter on 2021/5/17.
//
#ifndef RM_MANUAL_INCLUDE_RM_MANUAL_REFEREE_H_
#define RM_MANUAL_INCLUDE_RM_MANUAL_REFEREE_H_

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

#include "protocol.h"

namespace power_manager {
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
} // namespace power_manager

namespace referee {
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
  InteractiveData student_interactive_data_;
  GraphicDataStruct graphic_data_struct_;
  RobotInteractiveData robot_interactive_data_;
  RobotCommand robot_command_;
};

class Referee {
 public:
  Referee(ros::NodeHandle nh) {
    nh_ = nh;
  }
  void init();
  void read();

  double getActualBulletSpeed(int shoot_speed) const;
  double getUltimateBulletSpeed(int shoot_speed) const;

  ros::NodeHandle nh_;
  ros::Publisher referee_pub_;
  ros::Publisher power_manager_pub_;

  rm_msgs::Referee referee_pub_data_;
  rm_msgs::PowerManagerData power_manager_pub_data_;
  RefereeData referee_data_{};
  power_manager::PowerManagerData power_manager_data_;

  bool is_open_ = false;

  int robot_id_ = 0;
  int client_id_ = 0;

  void drawCircle(int center_x, int center_y, int radius, int picture_id, GraphicColorType color, uint8_t operate_type);
  void drawString(int x, int y, int picture_id, std::string data, GraphicColorType color, uint8_t operate_type);
  void sendInteractiveData(int data_cmd_id, int receiver_id, unsigned char data);

  void publishData();
 private:
  int unpack(uint8_t *rx_data);
  void pack(uint8_t *tx_buffer, uint8_t *data, int cmd_id, int len);

  void getRobotId();

  serial::Serial serial_;
  const std::string serial_port_ = "/dev/usbReferee";
  const int kUnpackLength = 256;
  const int kProtocolFrameLength = 128, kProtocolHeaderLength = 5, kProtocolCmdIdLength = 2, kProtocolTailLength = 2;
  std::vector<uint8_t> rx_data_;

  ros::Time last_get_referee_data_ = ros::Time::now();
};

// CRC verification
uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8);
uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length);
void appendCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t getCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);
void appendCRC16CheckSum(unsigned char *pchMessage, unsigned int dwLength);
} // namespace referee

#endif //RM_MANUAL_INCLUDE_RM_MANUAL_REFEREE_H_