//
// Created by luohx on 20-2-19.
//
#include "rm_manual/referee.h"
#include <ros/ros.h>

void Referee::init(ros::NodeHandle nh) {
  serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
  rx_data_.insert(rx_data_.begin(), kUnpackLength, 0);
  try {
    serial_.setPort(serial_port_);
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
  } catch (std::exception &e) {
    ROS_ERROR("Cannot set serial port of referee system, check whether the serial library is installed.");
    return;
  }

  if (!serial_.isOpen()) {
    try {
      serial_.open();
      is_open_ = true;
    } catch (serial::IOException &e) {
      ROS_WARN("Referee system serial cannot open [%s]", e.what());
    }
  }
  if (is_open_) {
    ROS_INFO("Referee serial open successfully.");
    referee_unpack_obj.index = 0;
    referee_unpack_obj.unpack_step = kStepHeaderSof;
  }
  nh_ = nh;
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>(
      "/dbus_data", 10, &Referee::dbusDataCallback, this);

  tf_listener_ = new tf2_ros::TransformListener(tf_);
}

/**
 * Send data to client UI
 */
void Referee::run() {
  uint8_t graph_operate_type;
  char power_string[30];
  float power_float;
  ros::Time now = ros::Time::now();
  geometry_msgs::TransformStamped gimbal_transformStamped;

  if (robot_id_ != 0 && robot_id_ != kRedSentry && robot_id_ != kBlueSentry) {
    if (dbus_data_.key_g) {
      if (now - last_press_time_g_ < ros::Duration(0.5)) dbus_data_.key_g = false;
      else last_press_time_g_ = now;
    }
    if (dbus_data_.key_r) {
      if (now - last_press_time_r_ < ros::Duration(0.5)) dbus_data_.key_r = false;
      else last_press_time_r_ = now;
    }
    if (dbus_data_.key_q) {
      if (now - last_press_time_q_ < ros::Duration(0.5)) dbus_data_.key_q = false;
      else last_press_time_q_ = now;
    }
    if (dbus_data_.key_c) {
      if (now - last_press_time_c_ < ros::Duration(0.5)) dbus_data_.key_c = false;
      else last_press_time_c_ = now;
    }

    if (dbus_data_.key_ctrl && dbus_data_.key_q) {
      is_chassis_passive_ = true;
      is_gimbal_passive_ = true;
      is_shooter_passive_ = true;
      chassis_update_flag_ = true;
      gimbal_update_flag_ = true;
      shooter_update_flag_ = true;
    }
    if (dbus_data_.key_ctrl && dbus_data_.key_w) {
      is_chassis_passive_ = false;
      is_gimbal_passive_ = false;
      is_shooter_passive_ = false;
      chassis_update_flag_ = true;
      gimbal_update_flag_ = true;
      shooter_update_flag_ = true;
    }

    if (dbus_data_.key_g) {
      gyro_flag_ = !gyro_flag_;
      chassis_update_flag_ = true;
    }
    if (dbus_data_.key_r) {
      twist_flag_ = !twist_flag_;
      chassis_update_flag_ = true;
    }
    if (!is_shooter_passive_ && dbus_data_.key_q) {
      burst_flag_ = !burst_flag_;
      shooter_update_flag_ = true;
    }
    if (dbus_data_.key_c) {
      only_attack_base_flag_ = !only_attack_base_flag_;
      attack_mode_update_flag_ = true;
    }
    if (dbus_data_.key_x) {
      graph_operate_type = kAdd;
      chassis_update_flag_ = true;
      gimbal_update_flag_ = true;
      shooter_update_flag_ = true;
      attack_mode_update_flag_ = true;
    } else {
      graph_operate_type = kUpdate;
    }

/*
    // get armors' position
    try {
      gimbal_transformStamped = this->tf_.lookupTransform("yaw", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      //ROS_ERROR("%s",ex.what());
    }
    quatToRPY(gimbal_transformStamped.transform.rotation, roll, pitch, yaw);

    // update information of armor
    if (referee_data_.robot_hurt_.armor_id == 0) {
      if (referee_data_.robot_hurt_.hurt_type == 0x0) {
        drawCircle((int) (960 + 340 * sin(0 - yaw)), (int) (540 + 340 * cos(0 - yaw)), 50, 5, kPurple, kAdd);
      } else if (referee_data_.robot_hurt_.hurt_type == 0x5) {
        drawCircle((int) (960 + 340 * sin(0 - yaw)), (int) (540 + 340 * cos(0 - yaw)), 50, 5, kYellow, kAdd);
      }
      last_hurt_id0_ = now;
      referee_data_.robot_hurt_.hurt_type = 0x9;
      referee_data_.robot_hurt_.armor_id = 9;
    }
    if (referee_data_.robot_hurt_.armor_id == 1) {
      if (referee_data_.robot_hurt_.hurt_type == 0x0) {
        drawCircle((int) (960 + 340 * sin(3 * M_PI_2 - yaw)), (int) (540 + 340 * cos(3 * M_PI_2 - yaw)),
                   50, 6, kPurple, kAdd);
      } else if (referee_data_.robot_hurt_.hurt_type == 0x5) {
        drawCircle((int) (960 + 340 * sin(3 * M_PI_2 - yaw)), (int) (540 + 340 * cos(3 * M_PI_2 - yaw)),
                   50, 6, kYellow, kAdd);
      }
      last_hurt_id1_ = now;
      referee_data_.robot_hurt_.hurt_type = 0x9;
      referee_data_.robot_hurt_.armor_id = 9;
    }
    if (referee_data_.robot_hurt_.armor_id == 2) {
      if (referee_data_.robot_hurt_.hurt_type == 0x0) { // bullet damage
        drawCircle((int) (960 + 340 * sin(M_PI - yaw)), (int) (540 + 340 * cos(M_PI - yaw)), 50, 7, kPurple, kAdd);
      } else if (referee_data_.robot_hurt_.hurt_type == 0x5) { // hit damage
        drawCircle((int) (960 + 340 * sin(M_PI - yaw)), (int) (540 + 340 * cos(M_PI - yaw)), 50, 7, kYellow, kAdd);
      }
      last_hurt_id2_ = now;
      referee_data_.robot_hurt_.hurt_type = 0x9;
      referee_data_.robot_hurt_.armor_id = 9;
    }
    if (referee_data_.robot_hurt_.armor_id == 3) {
      if (referee_data_.robot_hurt_.hurt_type == 0x0) {
        drawCircle((int) (960 + 340 * sin(M_PI_2 - yaw)), (int) (540 + 340 * cos(M_PI_2 - yaw)), 50, 8, kPurple, kAdd);
      } else if (referee_data_.robot_hurt_.hurt_type == 0x5) {
        drawCircle((int) (960 + 340 * sin(M_PI_2 - yaw)), (int) (540 + 340 * cos(M_PI_2 - yaw)), 50, 8, kYellow, kAdd);
      }
      last_hurt_id3_ = now;
      referee_data_.robot_hurt_.hurt_type = 0x9;
      referee_data_.robot_hurt_.armor_id = 9;
    }

    if (now - last_hurt_id0_ > ros::Duration(0.5)) {
      drawCircle((int) (960 + 340 * sin(0 - yaw)), (int) (540 + 340 * cos(0 - yaw)),
                 50, 5, kGreen, kDelete);
    }
    if (now - last_hurt_id1_ > ros::Duration(0.5)) {
      drawCircle((int) (960 + 340 * sin(3 * M_PI_2 - yaw)), (int) (540 + 340 * cos(3 * M_PI_2 - yaw)),
                 50, 6, kGreen, kDelete);
    }
    if (now - last_hurt_id2_ > ros::Duration(0.5)) {
      drawCircle((int) (960 + 340 * sin(M_PI - yaw)), (int) (540 + 340 * cos(M_PI - yaw)),
                 50, 7, kGreen, kDelete);
    }
    if (now - last_hurt_id3_ > ros::Duration(0.5)) {
      drawCircle((int) (960 + 340 * sin(M_PI_2 - yaw)), (int) (540 + 340 * cos(M_PI_2 - yaw)),
                 50, 8, kGreen, kDelete);
    }
*/

    power_float = power_manager_data_.parameters[3] * 100;
    sprintf(power_string, "Cap: %1.0f%%", power_float);
    if (power_float >= 60)
      drawString(910, 100, 4, kGreen, graph_operate_type, power_string);
    else if (power_float < 60 && power_float >= 30)
      drawString(910, 100, 4, kYellow, graph_operate_type, power_string);
    else if (power_float < 30)
      drawString(910, 100, 4, kOrange, graph_operate_type, power_string);

    if (chassis_update_flag_) {
      if (!is_chassis_passive_) {
        if (twist_flag_)
          drawString(1470, 790, 1, kYellow, graph_operate_type, "chassis:twist");
        else if (gyro_flag_)
          drawString(1470, 790, 1, kYellow, graph_operate_type, "chassis:gyro");
        else
          drawString(1470, 790, 1, kYellow, graph_operate_type, "chassis:follow");
      } else {
        drawString(1470, 790, 1, kYellow, graph_operate_type, "chassis:passive");
      }
      chassis_update_flag_ = false;
    }
    if (gimbal_update_flag_) {
      if (!is_gimbal_passive_) {
        if (dbus_data_.p_r)
          drawString(1470, 740, 2, kYellow, graph_operate_type, "gimbal:track");
        else
          drawString(1470, 740, 2, kYellow, graph_operate_type, "gimbal:rate");
      } else {
        drawString(1470, 740, 2, kYellow, graph_operate_type, "gimbal:passive");
      }
      gimbal_update_flag_ = false;
    }
    if (shooter_update_flag_) {
      if (!is_shooter_passive_) {
        if (burst_flag_)
          drawString(1470, 690, 3, kYellow, graph_operate_type, "shooter:burst");
        else
          drawString(1470, 690, 3, kYellow, graph_operate_type, "shooter:normal");
      } else {
        drawString(1470, 690, 3, kYellow, graph_operate_type, "shooter:passive");
      }
      shooter_update_flag_ = false;
    }
    if (attack_mode_update_flag_) {
      if (only_attack_base_flag_) {
        drawString(1470, 640, 4, kYellow, graph_operate_type, "target:base");
      } else {
        drawString(1470, 640, 4, kYellow, graph_operate_type, "target:all");
      }
      attack_mode_update_flag_ = false;
    }
  }

}

/******************* Receive data from referee system *************************/
void Referee::read() {
  std::vector<uint8_t> rx_buffer;
  std::vector<uint8_t> temp_buffer;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
  int rx_len;

  if (is_open_) {
    if (serial_.waitReadable()) {
      try {
        rx_len = serial_.available();
        serial_.read(rx_buffer, rx_len);
      } catch (serial::IOException &e) {
        ROS_ERROR("Referee system disconnect.");
        is_open_ = false;
        return;
      }

      // Unpack data from power manager
      power_manager_data_.read(rx_buffer);

      // Unpack data from referee system
      for (int kI = kUnpackLength; kI > rx_len; --kI) {
        temp_buffer.insert(temp_buffer.begin(), rx_data_[kI - 1]);
      }
      temp_buffer.insert(temp_buffer.end(), rx_buffer.begin(), rx_buffer.end());

      rx_data_.clear();
      rx_data_.insert(rx_data_.begin(), temp_buffer.begin(), temp_buffer.end());

      unpack(rx_data_);
    }

  } else {
    try {
      serial_.setPort(serial_port_);
      serial_.setBaudrate(115200);
      serial_.setTimeout(timeout);
    } catch (std::exception &e) {
      return;
    }

    if (!serial_.isOpen()) {
      try {
        serial_.open();
        is_open_ = true;
      } catch (serial::IOException &e) {}
    }

    if (is_open_) {
      ROS_INFO("Referee system reconnected.");
    }
  }

  getId();

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
  referee_pub_data_.stamp = ros::Time::now();

  power_manager_pub_data_.capacity = power_manager_data_.parameters[3] * 100;
  power_manager_pub_data_.chassis_power_buffer = power_manager_data_.parameters[2];
  power_manager_pub_data_.limit_power = power_manager_data_.parameters[1];
  power_manager_pub_data_.chassis_power = power_manager_data_.parameters[0];
  power_manager_pub_data_.stamp = ros::Time::now();

  referee_pub_.publish(referee_pub_data_);
  power_manager_pub_.publish(power_manager_pub_data_);
}

void Referee::unpack(const std::vector<uint8_t> &rx_buffer) {
  int num = 0, count = kUnpackLength;
  uint8_t byte;
  while (count) {
    byte = rx_buffer[num];

    switch (referee_unpack_obj.unpack_step) {
      case kStepHeaderSof: {
        if (byte == 0xA5) {
          referee_unpack_obj.unpack_step = kStepLengthLow;
          referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        } else {
          referee_unpack_obj.index = 0;
        }
      }
        break;
      case kStepLengthLow: {
        referee_unpack_obj.data_len = byte;
        referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        referee_unpack_obj.unpack_step = kStepLengthHigh;
      }
        break;
      case kStepLengthHigh: {
        referee_unpack_obj.data_len |= (byte << 8);
        referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        // Check for abnormal data length
        if (referee_unpack_obj.data_len < kProtocolFrameLength - kProtocolCmdIdLength - kProtocolTailLength) {
          referee_unpack_obj.unpack_step = kStepFrameSeq;
        } else {
          referee_unpack_obj.unpack_step = kStepHeaderSof;
          referee_unpack_obj.index = 0;
        }
      }
        break;
      case kStepFrameSeq: {
        referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        referee_unpack_obj.unpack_step = kStepHeaderCrc8;
      }
        break;
      case kStepHeaderCrc8: {
        referee_unpack_obj.protocol_packet[referee_unpack_obj.index] = byte;
        // Check if the check bit is being read
        if (referee_unpack_obj.index == (kProtocolHeaderLength - 1)) {
          if (verifyCRC8CheckSum(referee_unpack_obj.protocol_packet, kProtocolHeaderLength)) {
            referee_unpack_obj.unpack_step = kStepDataCrc16;
            referee_unpack_obj.index++;
          } else {
            referee_unpack_obj.unpack_step = kStepHeaderSof;
            referee_unpack_obj.index = 0;
          }
        } else {
          referee_unpack_obj.unpack_step = kStepHeaderSof;
          referee_unpack_obj.index = 0;
        }
      }
        break;
      case kStepDataCrc16: {
        // Read the remain data and check CRC16
        if (referee_unpack_obj.index
            < (kProtocolHeaderLength + kProtocolTailLength + kProtocolCmdIdLength + referee_unpack_obj.data_len)) {
          referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
        } else {
          if (verifyCRC16CheckSum(referee_unpack_obj.protocol_packet,
                                  kProtocolHeaderLength + kProtocolTailLength + kProtocolCmdIdLength
                                      + referee_unpack_obj.data_len)) {
            getData(referee_unpack_obj.protocol_packet);
            memset(referee_unpack_obj.protocol_packet, 0, sizeof(referee_unpack_obj.protocol_packet));
          }
          referee_unpack_obj.unpack_step = kStepHeaderSof;
          referee_unpack_obj.index = 0;
        }
      }
        break;
      default: {
        referee_unpack_obj.unpack_step = kStepHeaderSof;
        memset(referee_unpack_obj.protocol_packet, 0, sizeof(referee_unpack_obj.protocol_packet));
        referee_unpack_obj.index = 0;
        num = 0;
      }
        break;
    }
    num++;
    count--;
  }
  referee_unpack_obj.unpack_step = kStepHeaderSof;
  memset(referee_unpack_obj.protocol_packet, 0, sizeof(referee_unpack_obj.protocol_packet));
}

void Referee::getData(uint8_t *frame) {
  uint16_t cmd_id = 0;
  uint8_t index = 0;
  index += (sizeof(FrameHeaderStruct));
  memcpy(&cmd_id, frame + index, kProtocolCmdIdLength);

  index += kProtocolCmdIdLength;

  switch (cmd_id) {
    case kGameStatusCmdId: {
      memcpy(&referee_data_.game_status_, frame + index, sizeof(GameStatus));
      break;
    }
    case kGameResultCmdId: {
      memcpy(&referee_data_.game_result_, frame + index, sizeof(GameResult));
      break;
    }
    case kGameRobotHpCmdId: {
      memcpy(&referee_data_.game_robot_hp_, frame + index, sizeof(GameRobotHp));
      break;
    }
    case kDartStatusCmdId: {
      memcpy(&referee_data_.dart_status_, frame + index, sizeof(DartStatus));
      break;
    }
    case kIcraZoneStatusCmdId: {
      memcpy(&referee_data_.icra_buff_debuff_zone_status, frame + index, sizeof(IcraBuffDebuffZoneStatus));
      break;
    }
    case kFieldEventsCmdId: {
      memcpy(&referee_data_.event_data_, frame + index, sizeof(EventData));
      break;
    }
    case kSupplyProjectileActionCmdId: {
      memcpy(&referee_data_.supply_projectile_action_, frame + index, sizeof(SupplyProjectileAction));
      break;
    }
    case kRefereeWarningCmdId: {
      memcpy(&referee_data_.referee_warning_, frame + index, sizeof(RefereeWarning));
      break;
    }
    case kDartRemainingCmdId: {
      memcpy(&referee_data_.dart_remaining_time_, frame + index, sizeof(DartRemainingTime));
      break;
    }
    case kRobotStatusCmdId: {
      memcpy(&referee_data_.game_robot_status_, frame + index, sizeof(GameRobotStatus));
      break;
    }
    case kPowerHeatDataCmdId: {
      memcpy(&referee_data_.power_heat_data_, frame + index, sizeof(PowerHeatData));
      referee_data_.power_heat_data_.chassis_volt =
          referee_data_.power_heat_data_.chassis_volt / 1000;       //mV->V
      referee_data_.power_heat_data_.chassis_current =
          referee_data_.power_heat_data_.chassis_current / 1000;    //mA->A
      break;
    }
    case kRobotPosCmdId: {
      memcpy(&referee_data_.game_robot_pos_, frame + index, sizeof(GameRobotPos));
      break;
    }
    case kBuffCmdId: {
      memcpy(&referee_data_.buff_, frame + index, sizeof(Buff));
      break;
    }
    case kAerialRobotEnergyCmdId: {
      memcpy(&referee_data_.aerial_robot_energy_, frame + index, sizeof(AerialRobotEnergy));
      break;
    }
    case kRobotHurtCmdId: {
      memcpy(&referee_data_.robot_hurt_, frame + index, sizeof(RobotHurt));
      break;
    }
    case kShootDataCmdId: {
      memcpy(&referee_data_.shoot_data_, frame + index, sizeof(ShootData));
      break;
    }
    case kBulletRemainingCmdId: {
      memcpy(&referee_data_.bullet_remaining_, frame + index, sizeof(BulletRemaining));
      break;
    }
    case kRobotRfidStatusCmdId: {
      memcpy(&referee_data_.rfid_status_, frame + index, sizeof(RfidStatus));
      break;
    }
    case kDartClientCmdId: {
      memcpy(&referee_data_.dart_client_cmd_, frame + index, sizeof(DartClientCmd));
      break;
    }
    case kStudentInteractiveDataCmdId: {
      memcpy(&referee_data_.student_interactive_data_, frame + index, sizeof(StudentInteractiveData));
      break;
    }
    case kRobotCommandCmdId: {
      memcpy(&referee_data_.robot_command_, frame + index, sizeof(RobotCommand));
      break;
    }
    default: {
      ROS_WARN("Referee command ID not found.");
      break;
    }
  }
}

void Referee::getId() {
  if (robot_id_ == 0) {
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
    }
  }
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

/******************* Send data to referee system *************************/
/**
 * Draw a graph on client
 * @param robot_id
 * @param client_id
 * @param side
 * @param operate_type
 */
void Referee::drawCircle(int center_x, int center_y, int radius, int picture_name,
                         GraphicColorType color, uint8_t operate_type) {
  uint8_t tx_buffer[128] = {0};
  DrawClientGraphicData send_data;
  int index = 0;

  // Frame header
  send_data.tx_frame_header_.sof = 0xA5;
  send_data.tx_frame_header_.seq = 0;
  send_data.tx_frame_header_.data_length = sizeof(StudentInteractiveHeaderData) + sizeof(GraphicDataStruct);
  memcpy(tx_buffer, &send_data.tx_frame_header_, kProtocolHeaderLength);
  appendCRC8CheckSum(tx_buffer, kProtocolHeaderLength);
  index += kProtocolHeaderLength;

  // Command ID
  send_data.cmd_id_ = kStudentInteractiveDataCmdId;
  memcpy(tx_buffer + index, &send_data.cmd_id_, sizeof(kProtocolCmdIdLength));
  index += kProtocolCmdIdLength;

  // Data
  // Graph data header
  send_data.student_interactive_header_data_.data_cmd_id = kClientGraphicSingleCmdId;
  send_data.student_interactive_header_data_.send_ID = robot_id_;
  send_data.student_interactive_header_data_.receiver_ID = client_id_;
  memcpy(tx_buffer + index, &send_data.student_interactive_header_data_, sizeof(StudentInteractiveHeaderData));
  index += sizeof(StudentInteractiveHeaderData);

  // Graph data

  send_data.graphic_data_struct_.graphic_name[0] = (uint8_t) (picture_name & 0xff);
  send_data.graphic_data_struct_.graphic_name[1] = (uint8_t) ((picture_name >> 8) & 0xff);
  send_data.graphic_data_struct_.graphic_name[2] = (uint8_t) ((picture_name >> 16) & 0xff);

  send_data.graphic_data_struct_.start_x = center_x;
  send_data.graphic_data_struct_.start_y = center_y;
  send_data.graphic_data_struct_.radius = radius;

  send_data.graphic_data_struct_.operate_type = operate_type;
  send_data.graphic_data_struct_.graphic_type = 2; // circle
  send_data.graphic_data_struct_.layer = 0;
  send_data.graphic_data_struct_.color = color;
  send_data.graphic_data_struct_.width = 3;
  memcpy(tx_buffer + index, &send_data.graphic_data_struct_, sizeof(GraphicDataStruct));

  // Frame tail
  appendCRC16CheckSum(tx_buffer, sizeof(send_data));

  // Send
  try {
    serial_.write(tx_buffer, sizeof(send_data));
  } catch (serial::SerialException &e) {
    ROS_ERROR("Referee system disconnect.");
    is_open_ = false;
    return;
  }
}

void Referee::drawString(int x,
                         int y,
                         int picture_name,
                         GraphicColorType color,
                         uint8_t operate_type,
                         std::string data) {
  uint8_t tx_buffer[128] = {0};
  DrawClientCharData send_data;
  int index = 0;

  // Frame header
  send_data.tx_frame_header_.sof = 0xA5;
  send_data.tx_frame_header_.seq = 1;
  send_data.tx_frame_header_.data_length =
      sizeof(StudentInteractiveHeaderData) + sizeof(GraphicDataStruct) + 30;
  memcpy(tx_buffer, &send_data.tx_frame_header_, kProtocolHeaderLength);
  appendCRC8CheckSum(tx_buffer, kProtocolHeaderLength);
  index += kProtocolHeaderLength;

  // Command ID
  send_data.cmd_id_ = kStudentInteractiveDataCmdId;
  memcpy(tx_buffer + index, &send_data.cmd_id_, sizeof(kProtocolCmdIdLength));
  index += kProtocolCmdIdLength;

  // Data
  // Graph data header
  send_data.student_interactive_header_data_.data_cmd_id = kClientCharacterCmdId;
  send_data.student_interactive_header_data_.send_ID = robot_id_;
  send_data.student_interactive_header_data_.receiver_ID = client_id_;
  memcpy(tx_buffer + index, &send_data.student_interactive_header_data_, sizeof(StudentInteractiveHeaderData));
  index += sizeof(StudentInteractiveHeaderData);

  send_data.graphic_data_struct_.graphic_name[1] = (uint8_t) (picture_name & 0xff);
  send_data.graphic_data_struct_.start_x = x;
  send_data.graphic_data_struct_.start_y = y;

  send_data.graphic_data_struct_.graphic_name[0] = (uint8_t) ((picture_name >> 8) & 0xff);
  send_data.graphic_data_struct_.graphic_name[2] = (uint8_t) ((picture_name >> 16) & 0xff);
  send_data.graphic_data_struct_.graphic_type = 7; // char
  send_data.graphic_data_struct_.operate_type = operate_type;
  send_data.graphic_data_struct_.start_angle = 20; // char size
  send_data.graphic_data_struct_.end_angle = (int) data.size(); // string length
  send_data.graphic_data_struct_.width = 5; // line width
  send_data.graphic_data_struct_.layer = 0;
  send_data.graphic_data_struct_.color = color;
  memcpy(tx_buffer + index, &send_data.graphic_data_struct_, sizeof(GraphicDataStruct));
  index += sizeof(GraphicDataStruct);

  // Char data
  for (int kI = 0; kI < 30; ++kI) {
    if (kI < (int) data.size())
      send_data.data_[kI] = data[kI];
    else
      send_data.data_[kI] = ' ';
  }
  memcpy(tx_buffer + index, send_data.data_, 30);

  // Frame tail
  appendCRC16CheckSum(tx_buffer, sizeof(send_data));

  // Send
  try {
    serial_.write(tx_buffer, sizeof(send_data));
  } catch (serial::SerialException &e) {
    ROS_ERROR("Referee system disconnect.");
    is_open_ = false;
    return;
  }
}

void Referee::sendInteractiveData(int data_cmd_id, int receiver_id, const std::vector<uint8_t> &data) {
  uint8_t tx_buffer[128] = {0};
  SendInteractiveData send_data;
  int tx_len =
      kProtocolHeaderLength + kProtocolCmdIdLength + sizeof(StudentInteractiveHeaderData) + 113 + kProtocolTailLength;
  int index = 0;

  // Frame header
  send_data.tx_frame_header_.sof = 0xA5;
  send_data.tx_frame_header_.seq = 0;
  send_data.tx_frame_header_.data_length = sizeof(StudentInteractiveHeaderData) + data.size();
  memcpy(tx_buffer, &send_data.tx_frame_header_, kProtocolHeaderLength);
  appendCRC8CheckSum(tx_buffer, kProtocolHeaderLength);
  index += kProtocolHeaderLength;

  // Command ID
  send_data.cmd_id_ = kStudentInteractiveDataCmdId;
  memcpy(tx_buffer + index, &send_data.cmd_id_, sizeof(kProtocolCmdIdLength));
  index += kProtocolCmdIdLength;

  // Data
  // Interactive data header
  send_data.student_interactive_header_data_.data_cmd_id = data_cmd_id;
  send_data.student_interactive_header_data_.send_ID = robot_id_;
  send_data.student_interactive_header_data_.receiver_ID = receiver_id;
  memcpy(tx_buffer + index, &send_data.student_interactive_header_data_, sizeof(StudentInteractiveHeaderData));
  index += sizeof(StudentInteractiveHeaderData);
  // Interactive data
  for (int kI = 0; kI < 113; ++kI) {
    if (kI < (int) data.size())
      send_data.data_[kI] = data[kI];
    else
      send_data.data_[kI] = 0;
  }
  memcpy(tx_buffer + index, send_data.data_, 113);

  // Frame tail
  appendCRC16CheckSum(tx_buffer, tx_len);

  // Send
  try {
    serial_.write(tx_buffer, tx_len);
  } catch (serial::SerialException &e) {
    ROS_ERROR("Referee system disconnect.");
    is_open_ = false;
    return;
  }
}

/******************* CRC Verify *************************/
uint8_t getCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length, unsigned char ucCRC8) {
  unsigned char uc_index;
  while (dw_length--) {
    uc_index = ucCRC8 ^ (*pch_message++);
    ucCRC8 = CRC8_table[uc_index];
  }
  return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t verifyCRC8CheckSum(unsigned char *pch_message, unsigned int dw_length) {
  unsigned char ucExpected = 0;
  if ((pch_message == nullptr) || (dw_length <= 2)) {
    return 0;
  }
  ucExpected = getCRC8CheckSum(pch_message, dw_length - 1, kCrc8Init);
  return (ucExpected == pch_message[dw_length - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void appendCRC8CheckSum(unsigned char *pchMessage, unsigned int dwLength) {
  unsigned char ucCRC = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) return;
  ucCRC = getCRC8CheckSum((unsigned char *) pchMessage, dwLength - 1, kCrc8Init);
  pchMessage[dwLength - 1] = ucCRC;
}

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
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

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wExpected = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) {
    return 0;
  }
  wExpected = getCRC16CheckSum(pchMessage, dwLength - 2, kCrc16Init);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void appendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wCRC = 0;
  if ((pchMessage == nullptr) || (dwLength <= 2)) {
    return;
  }
  wCRC = getCRC16CheckSum((uint8_t *) pchMessage, dwLength - 2, kCrc16Init);
  pchMessage[dwLength - 2] = (uint8_t) (wCRC & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t) ((wCRC >> 8) & 0x00ff);
}

/***************************************** Power manager ****************************************************/
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
