//
// Created by peter on 2021/5/17.
//
#include "rm_manual/referee/referee.h"

namespace rm_manual {
void Referee::init() {
  serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
  rx_data_.insert(rx_data_.begin(), kUnpackLength, 0);
  try {
    serial_.setPort(serial_port_);
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
  } catch (std::exception &e) {
    ROS_ERROR(
        "Cannot set serial port of referee system, check whether the serial library is installed and try to use catkin clean");
    return;
  }
  if (!serial_.isOpen()) {
    try {
      serial_.open();
      is_open_ = true;
    } catch (serial::IOException &e) {
      ROS_ERROR("Cannot open referee port");
    }
  }
  if (is_open_) ROS_INFO("Referee system connect successfully");
}

// read data from referee
void Referee::read() {
  std::vector<uint8_t> rx_buffer;
  std::vector<uint8_t> temp_buffer;
  int rx_len, frame_len;

  if (is_open_) {
    if (serial_.waitReadable()) {
      try {
        rx_len = serial_.available();
        serial_.read(rx_buffer, rx_len);
      } catch (serial::IOException &e) {
        ROS_ERROR("Referee system disconnect, cannot read referee data");
        is_online_ = false;
        return;
      }

      for (int kI = kUnpackLength; kI > rx_len; --kI) {
        temp_buffer.insert(temp_buffer.begin(), rx_data_[kI - 1]);
      }
      temp_buffer.insert(temp_buffer.end(), rx_buffer.begin(), rx_buffer.end());
      rx_data_.clear();
      rx_data_.insert(rx_data_.begin(), temp_buffer.begin(), temp_buffer.end());

      for (int kI = 0; kI < kUnpackLength; ++kI) {
        if (rx_data_[kI] == 0xA5) {
          frame_len = unpack(&rx_data_[kI]);
          if (frame_len != -1) kI += frame_len;
        }
      }
      super_capacitor_.read(rx_buffer);
      getRobotId();
    }
  } else {
    init();
  }
  publishData();
}

int Referee::getShootSpeedLimit(int shoot_speed) const {
  if (is_open_) {
    if (robot_id_ == kBlueHero || robot_id_ == kRedHero) { // 42mm
      if (referee_data_.game_robot_status_.shooter_id1_42mm_speed_limit != 0)
        return referee_data_.game_robot_status_.shooter_id1_42mm_speed_limit;
    } else { // 17mm
      if (referee_data_.game_robot_status_.shooter_id1_17mm_speed_limit != 0)
        return referee_data_.game_robot_status_.shooter_id1_17mm_speed_limit;
    }
  }
  return shoot_speed;
}

int Referee::unpack(uint8_t *rx_data) {
  uint16_t cmd_id;
  int frame_len;
  FrameHeaderStruct frame_header;

  memcpy(&frame_header, rx_data, kProtocolHeaderLength);
  if (verifyCRC8CheckSum(rx_data, kProtocolHeaderLength) == true) {
    frame_len = frame_header.data_length + kProtocolHeaderLength + kProtocolCmdIdLength + kProtocolTailLength;;
    if (verifyCRC16CheckSum(rx_data, frame_len) == true) {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      switch (cmd_id) {
        case kGameStatusCmdId:memcpy(&referee_data_.game_status_, rx_data + 7, sizeof(GameStatus));
          break;
        case kGameResultCmdId:memcpy(&referee_data_.game_result_, rx_data + 7, sizeof(GameResult));
          break;
        case kGameRobotHpCmdId:memcpy(&referee_data_.game_robot_hp_, rx_data + 7, sizeof(GameRobotHp));
          break;
        case kDartStatusCmdId:memcpy(&referee_data_.dart_status_, rx_data + 7, sizeof(DartStatus));
          break;
        case kIcraZoneStatusCmdId:
          memcpy(&referee_data_.icra_buff_debuff_zone_status, rx_data + 7,
                 sizeof(IcraBuffDebuffZoneStatus));
          break;
        case kFieldEventsCmdId:memcpy(&referee_data_.event_data_, rx_data + 7, sizeof(EventData));
          break;
        case kSupplyProjectileActionCmdId:
          memcpy(&referee_data_.supply_projectile_action_, rx_data + 7,
                 sizeof(SupplyProjectileAction));
          break;
        case kRefereeWarningCmdId:memcpy(&referee_data_.referee_warning_, rx_data + 7, sizeof(RefereeWarning));
          break;
        case kDartRemainingCmdId:memcpy(&referee_data_.dart_remaining_time_, rx_data + 7, sizeof(DartRemainingTime));
          break;
        case kRobotStatusCmdId:memcpy(&referee_data_.game_robot_status_, rx_data + 7, sizeof(GameRobotStatus));
          break;
        case kPowerHeatDataCmdId:memcpy(&referee_data_.power_heat_data_, rx_data + 7, sizeof(PowerHeatData));
          referee_data_.power_heat_data_.chassis_volt *= 0.001; //mV->V
          referee_data_.power_heat_data_.chassis_current *= 0.001; //mA->A
          break;
        case kRobotPosCmdId:memcpy(&referee_data_.game_robot_pos_, rx_data + 7, sizeof(GameRobotPos));
          break;
        case kBuffCmdId:memcpy(&referee_data_.buff_, rx_data + 7, sizeof(Buff));
          break;
        case kAerialRobotEnergyCmdId:
          memcpy(&referee_data_.aerial_robot_energy_, rx_data + 7,
                 sizeof(AerialRobotEnergy));
          break;
        case kRobotHurtCmdId:memcpy(&referee_data_.robot_hurt_, rx_data + 7, sizeof(RobotHurt));
          break;
        case kShootDataCmdId:memcpy(&referee_data_.shoot_data_, rx_data + 7, sizeof(ShootData));
          break;
        case kBulletRemainingCmdId:memcpy(&referee_data_.bullet_remaining_, rx_data + 7, sizeof(BulletRemaining));
          break;
        case kRobotRfidStatusCmdId:memcpy(&referee_data_.rfid_status_, rx_data + 7, sizeof(RfidStatus));
          break;
        case kDartClientCmdId:memcpy(&referee_data_.dart_client_cmd_, rx_data + 7, sizeof(DartClientCmd));
          break;
        case kStudentInteractiveDataCmdId:
          memcpy(&referee_data_.student_interactive_data_, rx_data + 7,
                 sizeof(InteractiveData));
          break;
        case kRobotCommandCmdId:memcpy(&referee_data_.robot_command_, rx_data + 7, sizeof(RobotCommand));
          break;
        default:ROS_WARN("Referee command ID not found.");
          break;
      }
      last_get_referee_data_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}

void Referee::getRobotId() {
  robot_id_ = referee_data_.game_robot_status_.robot_id;
  if (robot_id_ != kBlueSentry && robot_id_ != kRedSentry) {
    switch (robot_id_) {
      case kBlueHero:client_id_ = kBlueHeroClientId;
        break;
      case kBlueEngineer:client_id_ = kBlueEngineerClientId;
        break;
      case kBlueStandard1:client_id_ = kBlueStandard1ClientId;
        break;
      case kBlueStandard2:client_id_ = kBlueStandard2ClientId;
        break;
      case kBlueStandard3:client_id_ = kBlueStandard3ClientId;
        break;
      case kRedHero:client_id_ = kRedHeroClientId;
        break;
      case kRedEngineer:client_id_ = kRedEngineerClientId;
        break;
      case kRedStandard1:client_id_ = kRedStandard1ClientId;
        break;
      case kRedStandard2:client_id_ = kRedStandard2ClientId;
        break;
      case kRedStandard3:client_id_ = kRedStandard3ClientId;
        break;
    }
  }
}

void Referee::publishData() {
  if (robot_id_ == kRedHero || robot_id_ == kBlueHero) {
    referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_id1_42mm_cooling_heat;
    referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_id1_42mm_cooling_limit;
  } else {
    referee_pub_data_.shooter_heat = referee_data_.power_heat_data_.shooter_id1_17mm_cooling_heat;
    referee_pub_data_.shooter_heat_cooling_limit = referee_data_.game_robot_status_.shooter_id1_17mm_cooling_limit;
  }
  referee_pub_data_.chassis_volt = referee_data_.power_heat_data_.chassis_volt;
  referee_pub_data_.chassis_current = referee_data_.power_heat_data_.chassis_current;
  referee_pub_data_.chassis_power = referee_data_.power_heat_data_.chassis_power;
  referee_pub_data_.chassis_power_buffer = referee_data_.power_heat_data_.chassis_power_buffer;
  referee_pub_data_.robot_hp = referee_data_.game_robot_status_.remain_HP;
  referee_pub_data_.hurt_armor_id = referee_data_.robot_hurt_.armor_id;
  referee_pub_data_.hurt_type = referee_data_.robot_hurt_.hurt_type;
  referee_pub_data_.bullet_speed = referee_data_.shoot_data_.bullet_speed;
  referee_pub_data_.stamp = last_get_referee_data_;

  super_capacitor_pub_data_.capacity = super_capacitor_.parameters[3] * 100;
  super_capacitor_pub_data_.chassis_power_buffer = super_capacitor_.parameters[2];
  super_capacitor_pub_data_.limit_power = super_capacitor_.parameters[1];
  super_capacitor_pub_data_.chassis_power = super_capacitor_.parameters[0];
  super_capacitor_pub_data_.stamp = super_capacitor_.last_get_capacitor_data_;

  referee_pub_.publish(referee_pub_data_);
  super_capacitor_pub_.publish(super_capacitor_pub_data_);
}

void Referee::displayCapInfo(GraphicOperateType graph_operate_type) {
  char power_string[30];
  float power_float;

  power_float = super_capacitor_.parameters[3] * 100;
  sprintf(power_string, "Cap: %1.0f%%", power_float);
  if (power_float >= 60)
    drawString(910, 100, 0, power_string, kGreen, graph_operate_type);
  else if (power_float < 60 && power_float >= 30)
    drawString(910, 100, 0, power_string, kYellow, graph_operate_type);
  else if (power_float < 30)
    drawString(910, 100, 0, power_string, kOrange, graph_operate_type);
}

void Referee::displayChassisInfo(uint8_t chassis_mode, bool unlimit_flag, GraphicOperateType graph_operate_type) {
  GraphicColorType color = unlimit_flag ? kOrange : kYellow;
  if (chassis_mode == rm_msgs::ChassisCmd::PASSIVE)
    drawString(1470, 790, 1, "chassis:passive", color, graph_operate_type);
  else if (chassis_mode == rm_msgs::ChassisCmd::FOLLOW)
    drawString(1470, 790, 1, "chassis:follow", color, graph_operate_type);
  else if (chassis_mode == rm_msgs::ChassisCmd::GYRO)
    drawString(1470, 790, 1, "chassis:gyro", color, graph_operate_type);
}

void Referee::displayGimbalInfo(uint8_t gimbal_mode, GraphicOperateType graph_operate_type) {
  if (gimbal_mode == rm_msgs::GimbalCmd::PASSIVE)
    drawString(1470, 740, 2, "gimbal:passive", kYellow, graph_operate_type);
  else if (gimbal_mode == rm_msgs::GimbalCmd::RATE)
    drawString(1470, 740, 2, "gimbal:rate", kYellow, graph_operate_type);
  else if (gimbal_mode == rm_msgs::GimbalCmd::TRACK)
    drawString(1470, 740, 2, "gimbal:track", kYellow, graph_operate_type);
}

void Referee::displayShooterInfo(uint8_t shooter_mode, bool burst_flag, GraphicOperateType graph_operate_type) {
  GraphicColorType color = burst_flag ? kOrange : kYellow;
  if (shooter_mode == rm_msgs::ShootCmd::PASSIVE)
    drawString(1470, 690, 3, "shooter:passive", color, graph_operate_type);
  else if (shooter_mode == rm_msgs::ShootCmd::READY)
    drawString(1470, 690, 3, "shooter:ready", color, graph_operate_type);
  else if (shooter_mode == rm_msgs::ShootCmd::PUSH)
    drawString(1470, 690, 3, "shooter:push", color, graph_operate_type);
  else if (shooter_mode == rm_msgs::ShootCmd::STOP)
    drawString(1470, 690, 3, "shooter:stop", color, graph_operate_type);
}

void Referee::displayAttackTargetInfo(bool attack_base_flag, GraphicOperateType graph_operate_type) {
  if (attack_base_flag) drawString(1470, 640, 4, "target:base", kYellow, graph_operate_type);
  else drawString(1470, 640, 4, "target:all", kYellow, graph_operate_type);
}

void Referee::displayArmorInfo(double yaw2baselink, const ros::Time &time) {
  if (referee_data_.robot_hurt_.hurt_type == 0x0) {
    if (referee_data_.robot_hurt_.armor_id == 0) {
      drawCircle((int) (960 + 340 * sin(0 + yaw2baselink)), (int) (540 + 340 * cos(0 + yaw2baselink)),
                 50, 5, kYellow, kAdd);
      last_update_armor0_time_ = time;
    } else if (referee_data_.robot_hurt_.armor_id == 1) {
      drawCircle((int) (960 + 340 * sin(3 * M_PI_2 + yaw2baselink)), (int) (540 + 340 * cos(3 * M_PI_2 + yaw2baselink)),
                 50, 6, kYellow, kAdd);
      last_update_armor1_time_ = time;
    } else if (referee_data_.robot_hurt_.armor_id == 2) {
      drawCircle((int) (960 + 340 * sin(M_PI + yaw2baselink)), (int) (540 + 340 * cos(M_PI + yaw2baselink)),
                 50, 7, kYellow, kAdd);
      last_update_armor2_time_ = time;
    } else if (referee_data_.robot_hurt_.armor_id == 3) {
      drawCircle((int) (960 + 340 * sin(M_PI_2 + yaw2baselink)), (int) (540 + 340 * cos(M_PI_2 + yaw2baselink)),
                 50, 8, kYellow, kAdd);
      last_update_armor3_time_ = time;
    }
    referee_data_.robot_hurt_.hurt_type = 0x9;
    referee_data_.robot_hurt_.armor_id = 9;
  }

  if (time - last_update_armor0_time_ > ros::Duration(0.5))
    drawCircle(0, 0, 0, 5, kYellow, kDelete);
  if (time - last_update_armor1_time_ > ros::Duration(0.5))
    drawCircle(0, 0, 0, 6, kYellow, kDelete);
  if (time - last_update_armor2_time_ > ros::Duration(0.5))
    drawCircle(0, 0, 0, 7, kYellow, kDelete);
  if (time - last_update_armor3_time_ > ros::Duration(0.5))
    drawCircle(0, 0, 0, 8, kYellow, kDelete);
}

void Referee::drawCircle(int center_x, int center_y, int radius, int picture_id,
                         GraphicColorType color, GraphicOperateType operate_type) {
  uint8_t tx_buffer[128] = {0};
  uint8_t tx_data[sizeof(ClientGraphicData)] = {0};
  auto client_graph_data = (ClientGraphicData *) tx_data;
  int tx_len = kProtocolHeaderLength + kProtocolCmdIdLength + sizeof(ClientGraphicData) + kProtocolTailLength;

  client_graph_data->student_interactive_header_data_.data_cmd_id = kClientGraphicSingleCmdId;
  client_graph_data->student_interactive_header_data_.send_ID = robot_id_;
  client_graph_data->student_interactive_header_data_.receiver_ID = client_id_;
  client_graph_data->graphic_data_struct_.graphic_name[0] = (uint8_t) (picture_id & 0xff);
  client_graph_data->graphic_data_struct_.graphic_name[1] = (uint8_t) ((picture_id >> 8) & 0xff);
  client_graph_data->graphic_data_struct_.graphic_name[2] = (uint8_t) ((picture_id >> 16) & 0xff);
  client_graph_data->graphic_data_struct_.start_x = center_x;
  client_graph_data->graphic_data_struct_.start_y = center_y;
  client_graph_data->graphic_data_struct_.radius = radius;
  client_graph_data->graphic_data_struct_.operate_type = operate_type;
  client_graph_data->graphic_data_struct_.graphic_type = 2; // circle
  client_graph_data->graphic_data_struct_.layer = 0;
  client_graph_data->graphic_data_struct_.color = color;
  client_graph_data->graphic_data_struct_.width = 3;
  pack(tx_buffer, tx_data, kStudentInteractiveDataCmdId, sizeof(ClientGraphicData));

  try {
    serial_.write(tx_buffer, tx_len);
  } catch (serial::PortNotOpenedException &e) {
    ROS_ERROR("Cannot open referee port, fail to draw UI");
    is_open_ = false;
    return;
  }
}

void Referee::drawString(int x, int y, int picture_id, std::string data,
                         GraphicColorType color, GraphicOperateType operate_type) {
  uint8_t tx_buffer[128] = {0};
  uint8_t tx_data[sizeof(ClientCharData)] = {0};
  auto client_char_data = (ClientCharData *) tx_data;
  int tx_len = kProtocolHeaderLength + kProtocolCmdIdLength + sizeof(ClientCharData) + kProtocolTailLength;

  client_char_data->student_interactive_header_data_.data_cmd_id = kClientCharacterCmdId;
  client_char_data->student_interactive_header_data_.send_ID = robot_id_;
  client_char_data->student_interactive_header_data_.receiver_ID = client_id_;
  client_char_data->graphic_data_struct_.graphic_name[0] = (uint8_t) ((picture_id >> 8) & 0xff);
  client_char_data->graphic_data_struct_.graphic_name[1] = (uint8_t) (picture_id & 0xff);
  client_char_data->graphic_data_struct_.graphic_name[2] = (uint8_t) ((picture_id >> 16) & 0xff);
  client_char_data->graphic_data_struct_.start_x = x;
  client_char_data->graphic_data_struct_.start_y = y;
  client_char_data->graphic_data_struct_.graphic_type = 7; // char
  client_char_data->graphic_data_struct_.operate_type = operate_type;
  client_char_data->graphic_data_struct_.start_angle = 20; // char size
  client_char_data->graphic_data_struct_.end_angle = (int) data.size(); // string length
  client_char_data->graphic_data_struct_.width = 5; // line width
  client_char_data->graphic_data_struct_.layer = 0;
  client_char_data->graphic_data_struct_.color = color;
  for (int kI = 0; kI < 30; ++kI) {
    if (kI < (int) data.size())
      client_char_data->data_[kI] = data[kI];
    else
      client_char_data->data_[kI] = ' ';
  }
  pack(tx_buffer, tx_data, kStudentInteractiveDataCmdId, sizeof(ClientCharData));

  try {
    serial_.write(tx_buffer, tx_len);
  } catch (serial::PortNotOpenedException &e) {
    ROS_ERROR("Cannot open referee port, fail to draw UI");
    is_open_ = false;
    return;
  }
}

void Referee::sendInteractiveData(int data_cmd_id, int receiver_id, uint8_t data) {
  uint8_t tx_buffer[128] = {0};
  uint8_t tx_data[sizeof(InteractiveData)] = {0};
  auto student_interactive_data = (InteractiveData *) tx_data;
  int tx_len = kProtocolHeaderLength + kProtocolCmdIdLength + sizeof(InteractiveData) + kProtocolTailLength;

  student_interactive_data->student_interactive_header_data_.data_cmd_id = data_cmd_id;
  student_interactive_data->student_interactive_header_data_.send_ID = robot_id_;
  student_interactive_data->student_interactive_header_data_.receiver_ID = receiver_id;
  student_interactive_data->data = data;
  pack(tx_buffer, tx_data, kStudentInteractiveDataCmdId, sizeof(InteractiveData));

  try {
    serial_.write(tx_buffer, tx_len);
  } catch (serial::PortNotOpenedException &e) {
    ROS_ERROR("Cannot open referee port, fail to send command to sentry");
    is_open_ = false;
    return;
  }
}

void Referee::pack(uint8_t *tx_buffer, uint8_t *data, int cmd_id, int len) {
  memset(tx_buffer, 0, kProtocolFrameLength);
  auto *frame_header = (FrameHeaderStruct *) tx_buffer;

  frame_header->sof = 0xA5;
  frame_header->data_length = len;
  memcpy(&tx_buffer[kProtocolHeaderLength], (uint8_t *) &cmd_id, kProtocolCmdIdLength);
  appendCRC8CheckSum(tx_buffer, kProtocolHeaderLength);
  memcpy(&tx_buffer[kProtocolHeaderLength + kProtocolCmdIdLength], data, len);
  appendCRC16CheckSum(tx_buffer, kProtocolHeaderLength + kProtocolCmdIdLength + len + kProtocolTailLength);
}

uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char uc_crc_8) {
  unsigned char uc_index;
  while (dw_length--) {
    uc_index = uc_crc_8 ^ (*pch_message++);
    uc_crc_8 = kCrc8Table[uc_index];
  }
  return (uc_crc_8);
}

uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length) {
  unsigned char uc_expected = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) return 0;
  uc_expected = getCRC8CheckSum(pch_message, dw_length - 1, kCrc8Init);
  return (uc_expected == pch_message[dw_length - 1]);
}

void appendCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length) {
  unsigned char uc_crc = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) return;
  uc_crc = getCRC8CheckSum((unsigned char *) pch_message, dw_length - 1, kCrc8Init);
  pch_message[dw_length - 1] = uc_crc;
}

uint16_t getCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length, uint16_t w_crc) {
  uint8_t chData;
  if (pch_message == nullptr) return 0xFFFF;
  while (dw_length--) {
    chData = *pch_message++;
    (w_crc) = ((uint16_t) (w_crc) >> 8) ^ wCRC_table[((uint16_t) (w_crc) ^ (uint16_t) (chData)) & 0x00ff];
  }
  return w_crc;
}

uint32_t verifyCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length) {
  uint16_t w_expected = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) return 0;
  w_expected = getCRC16CheckSum(pch_message, dw_length - 2, kCrc16Init);
  return ((w_expected & 0xff) == pch_message[dw_length - 2]
      && ((w_expected >> 8) & 0xff) == pch_message[dw_length - 1]);
}

void appendCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length) {
  uint16_t wCRC = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) return;
  wCRC = getCRC16CheckSum((uint8_t *) pch_message, dw_length - 2, kCrc16Init);
  pch_message[dw_length - 2] = (uint8_t) (wCRC & 0x00ff);
  pch_message[dw_length - 1] = (uint8_t) ((wCRC >> 8) & 0x00ff);
}

void SuperCapacitor::read(const std::vector<uint8_t> &rx_buffer) {
  int count = 0;
  memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
  memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
  receive_buf_counter_ = 0;
  for (unsigned char kI : rx_buffer) {
    dtpReceivedCallBack(kI);
    count++;
    if (count >= (int) sizeof(receive_buffer_)) {
      memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
      memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
      receive_buf_counter_ = 0;
    }
  }
  if (parameters[0] >= 120) parameters[0] = 120;
  if (parameters[0] <= 0) parameters[0] = 0;
  if (parameters[2] >= 25) parameters[2] = 25;
  if (parameters[2] <= 0) parameters[2] = 0;
  if (parameters[3] >= 1) parameters[3] = 1;
  if (parameters[3] <= 0) parameters[3] = 0;
}

void SuperCapacitor::receiveCallBack(unsigned char package_id, unsigned char *data) {
  if (package_id == 0) {
    last_get_capacitor_data_ = ros::Time::now();
    parameters[0] = int16ToFloat((data[0] << 8) | data[1]);
    parameters[1] = int16ToFloat((data[2] << 8) | data[3]);
    parameters[2] = int16ToFloat((data[4] << 8) | data[5]);
    parameters[3] = int16ToFloat((data[6] << 8) | data[7]);
  }
}

void SuperCapacitor::dtpReceivedCallBack(unsigned char receive_byte) {
  unsigned char check_flag;
  unsigned int sof_pos, eof_pos, check_counter;

  receive_buffer_[receive_buf_counter_] = receive_byte;
  receive_buf_counter_ = receive_buf_counter_ + 1;
  check_flag = 0;
  sof_pos = 0;
  eof_pos = 0;
  check_counter = 0;
  while (true) {
    if (check_flag == 0 && receive_buffer_[check_counter] == 0xff) {
      check_flag = 1;
      sof_pos = check_counter;
    } else if (check_flag == 1 && receive_buffer_[check_counter] == 0xff) {
      eof_pos = check_counter;
      break;
    }
    if (check_counter >= (receive_buf_counter_ - 1)) break;
    else check_counter++;
  }                                                           // Find Package In Buffer

  if ((eof_pos - sof_pos) == 11) {
    unsigned int temp_var;
    unsigned char data_buffer[8] = {0};
    unsigned char valid_buffer[12] = {0};

    for (temp_var = 0; temp_var < 12; temp_var++)           // Copy Data To Another Buffer
      valid_buffer[temp_var] = receive_buffer_[sof_pos + temp_var];

    eof_pos++;
    memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
    for (temp_var = 0; temp_var < receive_buf_counter_ - eof_pos; temp_var++)
      ping_pong_buffer_[temp_var] = receive_buffer_[eof_pos + temp_var];
    receive_buf_counter_ = receive_buf_counter_ - eof_pos;
    memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
    for (temp_var = 0; temp_var < receive_buf_counter_; temp_var++)
      receive_buffer_[temp_var] = ping_pong_buffer_[temp_var];

    unsigned char pid_bit = valid_buffer[1] >> 4;           // Get The PID Bit
    if (pid_bit == ((~(valid_buffer[1] & 0x0f)) & 0x0f)) {   // PID Verify
      for (temp_var = 0; temp_var < 8; ++temp_var)
        data_buffer[temp_var] = valid_buffer[2 + temp_var];
      if (valid_buffer[10] != 0x00) {                   // Some Byte had been replace
        unsigned char temp_filter = 0x00;
        for (temp_var = 0; temp_var < 8; ++temp_var)
          if (((valid_buffer[10] & (temp_filter | (0x01 << temp_var))) >> temp_var)
              == 1)                                   // This Byte Need To Adjust
            data_buffer[temp_var] = 0xff;           // Adjust to 0xff
      }
      receiveCallBack(pid_bit, data_buffer);
    }
  } else if ((eof_pos - sof_pos) != 0 && eof_pos != 0) {
    memset(receive_buffer_, 0x00, sizeof(receive_buffer_));
    memset(ping_pong_buffer_, 0x00, sizeof(ping_pong_buffer_));
    receive_buf_counter_ = 0;
  }
}

float SuperCapacitor::int16ToFloat(unsigned short data0) {
  if (data0 == 0) return 0;
  float *fp32;
  unsigned int fInt32 = ((data0 & 0x8000) << 16) |
      (((((data0 >> 10) & 0x1f) - 0x0f + 0x7f) & 0xff) << 23)
      | ((data0 & 0x03FF) << 13);
  fp32 = (float *) &fInt32;
  return *fp32;
}

} // namespace rm_manual

