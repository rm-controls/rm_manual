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

void CapacitorUi::add(double data) {
  auto graph = graph_vector_.find("capacitor");
  if (graph != graph_vector_.end() && data != 0.) graph->second->add();
}

void CapacitorUi::update(const ros::Time &time, double data) {
  auto graph = graph_vector_.find("capacitor");
  if (graph != graph_vector_.end()) {
    setConfig(graph->second, data);
    graph->second->update(time, data);
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

}
