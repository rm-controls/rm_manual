//
// Created by peter on 2021/7/20.
//

#include "rm_manual/referee/ui.h"

namespace rm_manual {
template<class GraphType>
UiBase<GraphType>::UiBase(ros::NodeHandle &nh, Referee &referee, const std::string &ui_type) : referee_(referee) {
  XmlRpc::XmlRpcValue config_param;
  if (!nh.getParam(ui_type, config_param)) {
    ROS_ERROR("%s no defined (namespace %s)", ui_type.c_str(), nh.getNamespace().c_str());
    return;
  }
  try {
    for (int i = 0; i < (int) config_param.size(); ++i)
      graph_vector_.insert(std::pair<std::string, GraphType *>(config_param[i]["name"],
                                                               new GraphType(config_param[i]["data"], referee_)));
  } catch (XmlRpc::XmlRpcException &e) { ROS_ERROR("Wrong ui parameter: %s", e.getMessage().c_str()); }
}

void WarningUi::update(const ros::Time &time, const std::string &graph_name, bool state) {
  auto graph = graph_vector_.find(graph_name);
  if (graph != graph_vector_.end()) graph->second->update(time, state);
}

void ControllersUi::update(const std::string &graph_name, uint8_t mode, bool flag) {
  auto graph = graph_vector_.find(graph_name);
  if (graph != graph_vector_.end()) {
    updateConfig(graph_name, graph->second, mode, flag);
    graph->second->update(mode, flag);
  }
}

void ControllersUi::updateConfig(const std::string &name, ManualChangeGraph *graph, uint8_t mode, bool flag) {
  if (name == "chassis") graph->setContent(getChassisState(mode));
  else if (name == "gimbal") graph->setContent(getGimbalState(mode));
  else if (name == "shooter") graph->setContent(getShooterState(mode));
  if (flag) graph->setColor(rm_common::GraphColor::ORANGE);
  else graph->setColor(rm_common::GraphColor::YELLOW);
}

const std::string ControllersUi::getChassisState(uint8_t mode) {
  if (mode == rm_msgs::ChassisCmd::RAW) return "raw";
  else if (mode == rm_msgs::ChassisCmd::FOLLOW) return "follow";
  else if (mode == rm_msgs::ChassisCmd::GYRO) return "gyro";
  else if (mode == rm_msgs::ChassisCmd::TWIST) return "twist";
  else return "error";
}
const std::string ControllersUi::getGimbalState(uint8_t mode) {
  if (mode == rm_msgs::GimbalCmd::RATE) return "rate";
  else if (mode == rm_msgs::GimbalCmd::TRACK) return "track";
  else if (mode == rm_msgs::GimbalCmd::DIRECT) return "direct";
  else return "error";
}
const std::string ControllersUi::getShooterState(uint8_t mode) {
  if (mode == rm_msgs::ShootCmd::STOP) return "stop";
  else if (mode == rm_msgs::ShootCmd::READY) return "ready";
  else if (mode == rm_msgs::ShootCmd::PUSH) return "push";
  else return "error";
}

void CapacitorUi::add(double data) {
  auto graph = graph_vector_.find("capacitor");
  if (graph != graph_vector_.end() && data != 0.) graph->second->add();
}

void CapacitorUi::update(const ros::Time &time, double data) {
  auto graph = graph_vector_.find("capacitor");
  if (graph != graph_vector_.end()) {
    setConfig(*graph->second, data);
    graph->second->update(time, data);
  }
}

void CapacitorUi::setConfig(AutoChangeGraph &config, double data) {
  char data_str[30] = {' '};
  sprintf(data_str, "cap:%1.0f%%", data);
  config.setContent(data_str);
  if (data < 30.) config.setColor(rm_common::GraphColor::ORANGE);
  else if (data > 70.) config.setColor(rm_common::GraphColor::GREEN);
  else config.setColor(rm_common::GraphColor::YELLOW);
}

}
