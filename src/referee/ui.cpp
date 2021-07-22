//
// Created by peter on 2021/7/20.
//

#include "rm_manual/referee/ui.h"

namespace rm_manual {
UiBase::UiBase(ros::NodeHandle &nh, Referee &referee, const std::string &ui_type) : referee_(referee) {
  XmlRpc::XmlRpcValue config_param;
  if (!nh.getParam(ui_type, config_param)) {
    ROS_ERROR("%s no defined (namespace %s)", ui_type.c_str(), nh.getNamespace().c_str());
    return;
  }
  try {
    for (int i = 0; i < (int) config_param.size(); ++i)
      graph_vector_.insert(std::pair<std::string, Graph *>(config_param[i]["name"],
                                                           new Graph(config_param[i]["data"], referee_)));
  } catch (XmlRpc::XmlRpcException &e) { ROS_ERROR("Wrong ui parameter: %s", e.getMessage().c_str()); }
  for (auto graph:graph_vector_) graph.second->setOperation(rm_common::GraphOperation::DELETE);
}

void UiBase::add() {
  for (auto graph:graph_vector_) {
    graph.second->setOperation(rm_common::GraphOperation::ADD);
    graph.second->display();
  }
}

void StateUi::update(const std::string &graph_name, uint8_t mode, bool flag) {
  auto graph = graph_vector_.find(graph_name);
  if (graph != graph_vector_.end()) {
    updateConfig(graph_name, graph->second, mode, flag);
    graph->second->setOperation(rm_common::GraphOperation::UPDATE);
    graph->second->display();
  }
}

void StateUi::updateConfig(const std::string &name, Graph *graph, uint8_t mode, bool flag) {
  if (flag) graph->setColor(rm_common::GraphColor::ORANGE);
  else graph->setColor(rm_common::GraphColor::YELLOW);
  if (name == "chassis") graph->setContent(getChassisState(mode));
  else if (name == "gimbal") graph->setContent(getGimbalState(mode));
  else if (name == "shooter") graph->setContent(getShooterState(mode));
  else if (name == "target") {
    graph->setContent(getTargetState(mode));
    if (flag) graph->setColor(rm_common::GraphColor::PINK);
    else graph->setColor(rm_common::GraphColor::CYAN);
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
    graph.second->setOperation(rm_common::GraphOperation::UPDATE);
    graph.second->display();
  }
}
void AimUi::updatePosition(Graph *graph, int level) {
  graph->setStartX(graph->getStartXArray()[level]);
  graph->setStartY(graph->getStartYArray()[level]);
  graph->setEndX(graph->getEndXArray()[level]);
  graph->setEndY(graph->getEndYArray()[level]);
}

void CapacitorUi::add() {
  auto graph = graph_vector_.find("capacitor");
  if (graph != graph_vector_.end() && referee_.referee_data_.capacity_data.cap_power_ != 0.) {
    graph->second->setOperation(rm_common::GraphOperation::ADD);
    graph->second->display();
  }
}

void CapacitorUi::update(const ros::Time &time) {
  auto graph = graph_vector_.find("capacitor");
  if (graph != graph_vector_.end() && referee_.referee_data_.capacity_data.cap_power_ != 0.) {
    setConfig(graph->second, referee_.referee_data_.capacity_data.cap_power_ * 100);
    graph->second->setOperation(rm_common::GraphOperation::UPDATE);
    graph->second->display(time);
  }
}

void CapacitorUi::setConfig(Graph *graph, double data) {
  char data_str[30] = {' '};
  sprintf(data_str, "cap:%1.0f%%", data);
  graph->setContent(data_str);
  if (data < 30.) graph->setColor(rm_common::GraphColor::ORANGE);
  else if (data > 70.) graph->setColor(rm_common::GraphColor::GREEN);
  else graph->setColor(rm_common::GraphColor::YELLOW);
}

}
