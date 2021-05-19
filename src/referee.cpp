//
// Created by peter on 2021/5/17.
//
#include "rm_manual/referee.h"
#include <ros/ros.h>

namespace referee {
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
        is_open_ = false;
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
      power_manager_data_.read(rx_buffer);
      getRobotId();
    }
  } else {
    init();
  }
  publishData();
}

double Referee::getActualBulletSpeed(int shoot_speed) const {
  if (is_open_) {
    if (referee_data_.shoot_data_.bullet_speed != 0)
      return referee_data_.shoot_data_.bullet_speed;
  }
  return shoot_speed;
}

double Referee::getUltimateBulletSpeed(int shoot_speed) const {
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
        case kGameStatusCmdId: {
          memcpy(&referee_data_.game_status_, rx_data + 7, sizeof(GameStatus));
          break;
        }
        case kGameResultCmdId: {
          memcpy(&referee_data_.game_result_, rx_data + 7, sizeof(GameResult));
          break;
        }
        case kGameRobotHpCmdId: {
          memcpy(&referee_data_.game_robot_hp_, rx_data + 7, sizeof(GameRobotHp));
          break;
        }
        case kDartStatusCmdId: {
          memcpy(&referee_data_.dart_status_, rx_data + 7, sizeof(DartStatus));
          break;
        }
        case kIcraZoneStatusCmdId: {
          memcpy(&referee_data_.icra_buff_debuff_zone_status, rx_data + 7, sizeof(IcraBuffDebuffZoneStatus));
          break;
        }
        case kFieldEventsCmdId: {
          memcpy(&referee_data_.event_data_, rx_data + 7, sizeof(EventData));
          break;
        }
        case kSupplyProjectileActionCmdId: {
          memcpy(&referee_data_.supply_projectile_action_, rx_data + 7, sizeof(SupplyProjectileAction));
          break;
        }
        case kRefereeWarningCmdId: {
          memcpy(&referee_data_.referee_warning_, rx_data + 7, sizeof(RefereeWarning));
          break;
        }
        case kDartRemainingCmdId: {
          memcpy(&referee_data_.dart_remaining_time_, rx_data + 7, sizeof(DartRemainingTime));
          break;
        }
        case kRobotStatusCmdId: {
          memcpy(&referee_data_.game_robot_status_, rx_data + 7, sizeof(GameRobotStatus));
          break;
        }
        case kPowerHeatDataCmdId: {
          memcpy(&referee_data_.power_heat_data_, rx_data + 7, sizeof(PowerHeatData));
          referee_data_.power_heat_data_.chassis_volt =
              referee_data_.power_heat_data_.chassis_volt / 1000;       //mV->V
          referee_data_.power_heat_data_.chassis_current =
              referee_data_.power_heat_data_.chassis_current / 1000;    //mA->A
          break;
        }
        case kRobotPosCmdId: {
          memcpy(&referee_data_.game_robot_pos_, rx_data + 7, sizeof(GameRobotPos));
          break;
        }
        case kBuffCmdId: {
          memcpy(&referee_data_.buff_, rx_data + 7, sizeof(Buff));
          break;
        }
        case kAerialRobotEnergyCmdId: {
          memcpy(&referee_data_.aerial_robot_energy_, rx_data + 7, sizeof(AerialRobotEnergy));
          break;
        }
        case kRobotHurtCmdId: {
          memcpy(&referee_data_.robot_hurt_, rx_data + 7, sizeof(RobotHurt));
          break;
        }
        case kShootDataCmdId: {
          memcpy(&referee_data_.shoot_data_, rx_data + 7, sizeof(ShootData));
          break;
        }
        case kBulletRemainingCmdId: {
          memcpy(&referee_data_.bullet_remaining_, rx_data + 7, sizeof(BulletRemaining));
          break;
        }
        case kRobotRfidStatusCmdId: {
          memcpy(&referee_data_.rfid_status_, rx_data + 7, sizeof(RfidStatus));
          break;
        }
        case kDartClientCmdId: {
          memcpy(&referee_data_.dart_client_cmd_, rx_data + 7, sizeof(DartClientCmd));
          break;
        }
        case kStudentInteractiveDataCmdId: {
          memcpy(&referee_data_.student_interactive_data_, rx_data + 7, sizeof(InteractiveData));
          break;
        }
        case kRobotCommandCmdId: {
          memcpy(&referee_data_.robot_command_, rx_data + 7, sizeof(RobotCommand));
          break;
        }
        default: {
          ROS_WARN("Referee command ID not found.");
          break;
        }
      }
      last_get_referee_data_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}

void Referee::getRobotId() {
  robot_id_ = referee_data_.game_robot_status_.robot_id;
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
    default:ROS_WARN("Cannot get robot id");
      break;
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

  power_manager_pub_data_.capacity = power_manager_data_.parameters[3] * 100;
  power_manager_pub_data_.chassis_power_buffer = power_manager_data_.parameters[2];
  power_manager_pub_data_.limit_power = power_manager_data_.parameters[1];
  power_manager_pub_data_.chassis_power = power_manager_data_.parameters[0];
  power_manager_pub_data_.stamp = power_manager_data_.last_get_powermanager_data_;

  referee_pub_.publish(referee_pub_data_);
  power_manager_pub_.publish(power_manager_pub_data_);
}

void Referee::drawCircle(int center_x, int center_y, int radius, int picture_id,
                         GraphicColorType color, uint8_t operate_type) {
  uint8_t tx_buffer[128] = {0};
  uint8_t tx_data[sizeof(ClientGraphicData)] = {0};
  auto client_graph_data = (ClientGraphicData *) tx_data;
  int tx_len = kProtocolHeaderLength + kProtocolCmdIdLength + sizeof(ClientGraphicData) + kProtocolTailLength;

  // Data
  // Graph data header
  client_graph_data->student_interactive_header_data_.data_cmd_id = kClientGraphicSingleCmdId;
  client_graph_data->student_interactive_header_data_.send_ID = robot_id_;
  client_graph_data->student_interactive_header_data_.receiver_ID = client_id_;

  // Graph data
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

  // Send
  try {
    serial_.write(tx_buffer, tx_len);
  } catch (serial::PortNotOpenedException &e) {
    ROS_ERROR("Cannot open referee port, fail to draw UI");
    is_open_ = false;
    return;
  }
}

void Referee::drawString(int x, int y, int picture_id, std::string data, GraphicColorType color, uint8_t operate_type) {
  uint8_t tx_buffer[128] = {0};
  uint8_t tx_data[sizeof(ClientCharData)] = {0};
  auto client_char_data = (ClientCharData *) tx_data;
  int tx_len = kProtocolHeaderLength + kProtocolCmdIdLength + sizeof(ClientCharData) + kProtocolTailLength;

  // Data
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

  // Char data
  for (int kI = 0; kI < 30; ++kI) {
    if (kI < (int) data.size())
      client_char_data->data_[kI] = data[kI];
    else
      client_char_data->data_[kI] = ' ';
  }

  pack(tx_buffer, tx_data, kStudentInteractiveDataCmdId, sizeof(ClientCharData));

  // Send
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

  // Send
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

// CRC verify
uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8) {
  unsigned char uc_index;
  while (dw_length--) {
    uc_index = ucCRC8 ^ (*pch_message++);
    ucCRC8 = CRC8_table[uc_index];
  }
  return (ucCRC8);
}

uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length) {
  unsigned char ucExpected = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) {
    return 0;
  }
  ucExpected = getCRC8CheckSum(pch_message, dw_length - 1, kCrc8Init);
  return (ucExpected == pch_message[dw_length - 1]);
}

void appendCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength) {
  unsigned char ucCRC = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) return;
  ucCRC = getCRC8CheckSum((unsigned char *) pchMessage, dwLength - 1, kCrc8Init);
  pchMessage[dwLength - 1] = ucCRC;
}

uint16_t getCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
  uint8_t chData;
  if (pchMessage == nullptr) {
    return 0xFFFF;
  }
  while (dwLength--) {
    chData = *pchMessage++;
    (wCRC) = ((uint16_t) (wCRC) >> 8) ^ wCRC_table[((uint16_t) (wCRC) ^ (uint16_t) (chData)) & 0x00ff];
  }
  return wCRC;
}

uint32_t verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wExpected = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) {
    return 0;
  }
  wExpected = getCRC16CheckSum(pchMessage, dwLength - 2, kCrc16Init);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void appendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wCRC = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) {
    return;
  }
  wCRC = getCRC16CheckSum((uint8_t *) pchMessage, dwLength - 2, kCrc16Init);
  pchMessage[dwLength - 2] = (uint8_t) (wCRC & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t) ((wCRC >> 8) & 0x00ff);
}
} // namespace referee

namespace power_manager {
void PowerManagerData::read(const std::vector<uint8_t> &rx_buffer) {
  int count = 0;
  memset(Receive_Buffer, 0x00, sizeof(Receive_Buffer));
  memset(PingPong_Buffer, 0x00, sizeof(PingPong_Buffer));
  Receive_BufCounter = 0;
  for (unsigned char kI : rx_buffer) {
    DTP_Received_CallBack(kI);
    count++;
    if (count >= (int) sizeof(Receive_Buffer)) {
      memset(Receive_Buffer, 0x00, sizeof(Receive_Buffer));
      memset(PingPong_Buffer, 0x00, sizeof(PingPong_Buffer));
      Receive_BufCounter = 0;
    }
  }
  if (parameters[0] >= 120) parameters[0] = 120;
  if (parameters[0] <= 0) parameters[0] = 0;

  if (parameters[2] >= 25) parameters[2] = 25;
  if (parameters[2] <= 0) parameters[2] = 0;

  if (parameters[3] >= 1) parameters[3] = 1;
  if (parameters[3] <= 0) parameters[3] = 0;
}

void PowerManagerData::Receive_CallBack(unsigned char PID, unsigned char Data[8]) {
  if (PID == 0) {
    last_get_powermanager_data_ = ros::Time::now();
    parameters[0] = Int16ToFloat((Data[0] << 8) | Data[1]);
    parameters[1] = Int16ToFloat((Data[2] << 8) | Data[3]);
    parameters[2] = Int16ToFloat((Data[4] << 8) | Data[5]);
    parameters[3] = Int16ToFloat((Data[6] << 8) | Data[7]);
  }
}

void PowerManagerData::DTP_Received_CallBack(unsigned char Receive_Byte) {

  unsigned char CheckFlag;
  unsigned int SOF_Pos, EOF_Pos, CheckCounter;

  Receive_Buffer[Receive_BufCounter] = Receive_Byte;
  Receive_BufCounter = Receive_BufCounter + 1;

  CheckFlag = 0;
  SOF_Pos = 0;
  EOF_Pos = 0;
  CheckCounter = 0;
  while (true) {
    if (CheckFlag == 0 && Receive_Buffer[CheckCounter] == 0xff) {
      CheckFlag = 1;
      SOF_Pos = CheckCounter;
    } else if (CheckFlag == 1 && Receive_Buffer[CheckCounter] == 0xff) {
      EOF_Pos = CheckCounter;
      break;
    }
    if (CheckCounter >= (Receive_BufCounter - 1))
      break;
    else
      CheckCounter++;
  }                                                           //Find Package In Buffer


  if ((EOF_Pos - SOF_Pos) == 11) {
    unsigned int Temp_Var;
    unsigned char Data_Buffer[8] = {0};
    unsigned char Valid_Buffer[12] = {0};

    for (Temp_Var = 0; Temp_Var < 12; Temp_Var++)           //Copy Data To Another Buffer
      Valid_Buffer[Temp_Var] = Receive_Buffer[SOF_Pos + Temp_Var];

    EOF_Pos++;
    memset(PingPong_Buffer, 0x00, sizeof(PingPong_Buffer));
    for (Temp_Var = 0; Temp_Var < Receive_BufCounter - EOF_Pos; Temp_Var++)
      PingPong_Buffer[Temp_Var] = Receive_Buffer[EOF_Pos + Temp_Var];
    Receive_BufCounter = Receive_BufCounter - EOF_Pos;
    memset(Receive_Buffer, 0x00, sizeof(Receive_Buffer));
    for (Temp_Var = 0; Temp_Var < Receive_BufCounter; Temp_Var++)
      Receive_Buffer[Temp_Var] = PingPong_Buffer[Temp_Var];

    unsigned char PID_Bit = Valid_Buffer[1] >> 4;           //Get The PID Bit
    if (PID_Bit == ((~(Valid_Buffer[1] & 0x0f)) & 0x0f))    //PID Verify
    {
      for (Temp_Var = 0; Temp_Var < 8; ++Temp_Var)
        Data_Buffer[Temp_Var] = Valid_Buffer[2 + Temp_Var];
      if (Valid_Buffer[10] != 0x00)                       //Some Byte had been replace
      {
        unsigned char Temp_Filter = 0x00;
        for (Temp_Var = 0; Temp_Var < 8; ++Temp_Var)
          if (((Valid_Buffer[10] & (Temp_Filter | (0x01 << Temp_Var))) >> Temp_Var)
              == 1)                                   //This Byte Need To Adjust
            Data_Buffer[Temp_Var] = 0xff;           //Adjust to 0xff
      }
      Receive_CallBack(PID_Bit, Data_Buffer);
    }
  } else if ((EOF_Pos - SOF_Pos) != 0 && EOF_Pos != 0) {
    memset(Receive_Buffer, 0x00, sizeof(Receive_Buffer));
    memset(PingPong_Buffer, 0x00, sizeof(PingPong_Buffer));
    Receive_BufCounter = 0;
  }
}

float PowerManagerData::Int16ToFloat(unsigned short data0) {
  if (data0 == 0)
    return 0;
  float *fp32;
  unsigned int fInt32 = ((data0 & 0x8000) << 16) |
      (((((data0 >> 10) & 0x1f) - 0x0f + 0x7f) & 0xff) << 23)
      | ((data0 & 0x03FF) << 13);
  fp32 = (float *) &fInt32;
  return *fp32;
}
} // namespace power_manager

