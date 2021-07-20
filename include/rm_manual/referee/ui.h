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
  void getParam(const std::string &ui_type, ros::NodeHandle &nh) {
    XmlRpc::XmlRpcValue config_param;
    if (!nh.getParam(ui_type, config_param)) {
      ROS_ERROR("%s no defined (namespace %s)", ui_type.c_str(), nh.getNamespace().c_str());
      return;
    }
    for (int i = 0; i < (int) config_param.size(); ++i)
      graph_vector_.push_back(new GraphType(config_param[i], referee_));
  }
  Referee &referee_;
  std::vector<GraphType *> graph_vector_;
};

class TitleUi : public UiBase<UnChangeGraph> {
 public:
  explicit TitleUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee) {
    getParam("title", nh);
  }
  void add() { for (int i = 0; i < (int) graph_vector_.size(); ++i) graph_vector_[i]->add(); }
};

class WarningUi : public UiBase<AutoChangeGraph> {
 public:
  explicit WarningUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee) {
    getParam("warning", nh);
  }
  void update(const ros::Time &time, const std::string &graph_name, bool state) {
    for (int i = 0; i < (int) graph_vector_.size(); ++i)
      if (graph_vector_[i]->getName() == graph_name) graph_vector_[i]->update(time, state);
  }
};

class CapacitorUi : public UiBase<AutoChangeGraph> {
 public:
  explicit CapacitorUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee) {
    getParam("capacitor", nh);
  }
  void add(double data) {
    if (!graph_vector_.empty() && data != 0.) {
      setConfig(*graph_vector_[0], data);
      graph_vector_[0]->add();
    }
  }
  void update(const ros::Time &time, double data) {
    if (!graph_vector_.empty()) {
      setConfig(*graph_vector_[0], data);
      graph_vector_[0]->update(time, data);
    }
  }
 private:
  void setConfig(AutoChangeGraph &config, double data) {
    char data_str[30] = {' '};
    sprintf(data_str, "cap:%1.0f%%", data);
    config.setContent(data_str);
    if (data < 30.) config.setColor(rm_common::GraphColor::ORANGE);
    else if (data > 70.) config.setColor(rm_common::GraphColor::GREEN);
    else config.setColor(rm_common::GraphColor::YELLOW);
  }
};

class StateUi : public UiBase<ManualChangeGraph> {
 public:
  explicit StateUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee) {
    getParam("state", nh);
  }
  void add() {

  }
};
} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
