//
// Created by peter on 2021/5/24.
//

#ifndef RM_MANUAL_REFEREE_UI_H_
#define RM_MANUAL_REFEREE_UI_H_

#include "rm_manual/referee/referee.h"
#include "rm_manual/referee/graph.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>

namespace rm_manual {
template<class GraphType>
class UiBase {
 public:
  explicit UiBase(ros::NodeHandle &nh, Referee &referee) : referee_(referee) {}
 protected:
  std::vector<GraphType *> graph_vector_;
  Referee &referee_;
};

class TitleUi : public UiBase<UnChangeGraph> {
 public:
  explicit TitleUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee) {
    XmlRpc::XmlRpcValue title_config;
    if (!nh.getParam("title", title_config)) {
      ROS_ERROR("Title no defined (namespace %s)", nh.getNamespace().c_str());
      return;
    }
    for (int i = 0; i < (int) title_config.size(); ++i)
      graph_vector_.push_back(new UnChangeGraph(title_config[i], referee));
  }
  void add() { for (int i = 0; i < (int) graph_vector_.size(); ++i) graph_vector_[i]->add(); }
};

class WarningUi : public UiBase<AutoChangeGraph> {
 public:
  explicit WarningUi(ros::NodeHandle &nh, Referee &referee, const ros::Duration &duration) : UiBase(nh, referee) {
    XmlRpc::XmlRpcValue warning_config;
    if (!nh.getParam("warning", warning_config)) {
      ROS_ERROR("Warning no defined (namespace %s)", nh.getNamespace().c_str());
      return;
    }
    for (int i = 0; i < (int) warning_config.size(); ++i)
      graph_vector_.push_back(new AutoChangeGraph(warning_config[i], referee, duration));
  }
  void update(const ros::Time &time, const std::string &graph_name, bool state) {
    for (int i = 0; i < (int) graph_vector_.size(); ++i)
      if (graph_vector_[i]->getName() == graph_name) graph_vector_[i]->update(time, state);
  }
};

class CapacitorUi : public UiBase<AutoChangeGraph> {
 public:
  explicit CapacitorUi(ros::NodeHandle &nh, Referee &referee, const ros::Duration &duration) : UiBase(nh, referee) {
    XmlRpc::XmlRpcValue capacitor_config;
    if (!nh.getParam("capacitor", capacitor_config)) {
      ROS_ERROR("Capacitor no defined (namespace %s)", nh.getNamespace().c_str());
      return;
    }
    for (int i = 0; i < (int) capacitor_config.size(); ++i)
      graph_vector_.push_back(new AutoChangeGraph(capacitor_config[i], referee, duration));
  }
  void update(const ros::Time &time, double data) {
    if (!graph_vector_.empty()) {
      char data_str[30] = {' '};
      sprintf(data_str, "cap:%1.0f%%", data);
      graph_vector_[0]->setContent(data_str);
      if (data < 30.) graph_vector_[0]->setColor(rm_common::GraphColor::ORANGE);
      else if (data > 70.) graph_vector_[0]->setColor(rm_common::GraphColor::GREEN);
      else graph_vector_[0]->setColor(rm_common::GraphColor::YELLOW);
      graph_vector_[0]->update(time, data);
    }
  }
};

} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
