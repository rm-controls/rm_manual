//
// Created by peter on 2021/7/20.
//

#include "rm_manual/referee/ui.h"

namespace rm_manual {
void WarningUi::update(const std::string &graph_name, bool state, const ros::Time &time) {
  auto graph = graph_vector_.find(graph_name);
  if (graph != graph_vector_.end()) graph->second->update(time, state);
}

void StateUi::update(const std::string &graph_name, uint8_t mode, bool flag) {
  auto graph = graph_vector_.find(graph_name);
  if (graph != graph_vector_.end()) {
    updateConfig(graph_name, graph->second, mode, flag);
    graph->second->update(mode, flag);
  }
}

void StateUi::updateConfig(const std::string &name, ManualChangeGraph *config, uint8_t mode, bool flag) {
  if (flag) config->setColor(rm_common::GraphColor::ORANGE);
  else config->setColor(rm_common::GraphColor::YELLOW);
  if (name == "chassis") config->setContent(getChassisState(mode));
  else if (name == "gimbal") config->setContent(getGimbalState(mode));
  else if (name == "shooter") config->setContent(getShooterState(mode));
  else if (name == "target") {
    config->setContent(getTargetState(mode));
    if (flag) config->setColor(rm_common::GraphColor::PINK);
    else config->setColor(rm_common::GraphColor::CYAN);
  }
}

const std::string StateUi::getChassisState(uint8_t mode) {
  if (mode == rm_msgs::ChassisCmd::RAW) return "raw";
  else if (mode == rm_msgs::ChassisCmd::FOLLOW) return "follow";
  else if (mode == rm_msgs::ChassisCmd::GYRO) return "gyro";
  else if (mode == rm_msgs::ChassisCmd::TWIST) return "twist";
  else return "error";
}

const std::string StateUi::getGimbalState(uint8_t mode) {
  if (mode == rm_msgs::GimbalCmd::RATE) return "rate";
  else if (mode == rm_msgs::GimbalCmd::TRACK) return "track";
  else if (mode == rm_msgs::GimbalCmd::DIRECT) return "direct";
  else return "error";
}

const std::string StateUi::getShooterState(uint8_t mode) {
  if (mode == rm_msgs::ShootCmd::STOP) return "stop";
  else if (mode == rm_msgs::ShootCmd::READY) return "ready";
  else if (mode == rm_msgs::ShootCmd::PUSH) return "push";
  else return "error";
}

const std::string StateUi::getTargetState(uint8_t mode) {
  if (mode == rm_msgs::StatusChangeRequest::BUFF) return "buff";
  else if (mode == rm_msgs::StatusChangeRequest::ARMOR) return "armor";
  else return "error";
}

void AimUi::update() {
  if (referee_.referee_data_.game_robot_status_.robot_level_ >= 4) return;
  for (auto graph:graph_vector_) {
    updatePosition(graph.second, referee_.referee_data_.game_robot_status_.robot_level_);
    graph.second->update(referee_.referee_data_.game_robot_status_.robot_level_);
  }
}
void AimUi::updatePosition(AutoChangeGraph *graph, int level) {
  graph->setStartX(graph->getStartXArray()[level]);
  graph->setStartY(graph->getStartYArray()[level]);
  graph->setEndX(graph->getEndXArray()[level]);
  graph->setEndY(graph->getEndYArray()[level]);
}

void CapacitorUi::add() {
  auto graph = graph_vector_.find("capacitor");
  if (graph != graph_vector_.end() && referee_.referee_data_.capacity_data.buffer_power_ != 0.) graph->second->add();
}

void CapacitorUi::update(const ros::Time &time) {
  auto graph = graph_vector_.find("capacitor");
  if (graph != graph_vector_.end()) {
    setConfig(graph->second, referee_.referee_data_.capacity_data.buffer_power_);
    graph->second->update(time, referee_.referee_data_.capacity_data.buffer_power_);
  }
}

void CapacitorUi::setConfig(AutoChangeGraph *config, double data) {
  char data_str[30] = {' '};
  sprintf(data_str, "cap:%1.0f%%", data);
  config->setContent(data_str);
  if (data < 30.) config->setColor(rm_common::GraphColor::ORANGE);
  else if (data > 70.) config->setColor(rm_common::GraphColor::GREEN);
  else config->setColor(rm_common::GraphColor::YELLOW);
}

void WarningUi::updateArmorPosition(AutoChangeGraph *graph, int armor_id) {
  geometry_msgs::TransformStamped yaw2baselink;
  double roll, pitch, yaw;
  try { yaw2baselink = tf_.lookupTransform("yaw", "base_link", ros::Time(0)); }
  catch (tf2::TransformException &ex) {}
  quatToRPY(yaw2baselink.transform.rotation, roll, pitch, yaw);
  if (armor_id == 0 || armor_id == 2) {
    graph->setStartX((int) (960 + 340 * sin(armor_id * M_PI_2 + yaw)));
    graph->setStartY((int) (540 + 340 * cos(armor_id * M_PI_2 + yaw)));
  } else {
    graph->setStartX((int) (960 + 340 * sin(-armor_id * M_PI_2 + yaw)));
    graph->setStartY((int) (540 + 340 * cos(-armor_id * M_PI_2 + yaw)));
  }
}

uint8_t WarningUi::getArmorId(const std::string &name) {
  if (name == "armor0") return 0;
  else if (name == "armor1") return 1;
  else if (name == "armor2") return 2;
  else if (name == "armor3") return 3;
  return 9;
}

}
