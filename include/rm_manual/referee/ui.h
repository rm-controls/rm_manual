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
  explicit UiBase(ros::NodeHandle &nh, Referee &referee, const std::string &ui_type) : referee_(referee) {
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
 protected:
  Referee &referee_;
  std::map<std::string, GraphType *> graph_vector_;
};

class TitleUi : public UiBase<UnChangeGraph> {
 public:
  explicit TitleUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "title") {};
  void add() { for (auto graph:graph_vector_) graph.second->add(); }
};

class WarningUi : public UiBase<AutoChangeGraph> {
 public:
  explicit WarningUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "warning") {};
  void update(const std::string &graph_name, bool state, const ros::Time &time);
};

class StateUi : public UiBase<ManualChangeGraph> {
 public:
  explicit StateUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "state") {};
  void add() { for (auto graph:graph_vector_) graph.second->add(); }
  void update(const std::string &graph_name, uint8_t mode, bool flag = false);
 protected:
  void updateConfig(const std::string &name, ManualChangeGraph *graph, uint8_t mode, bool flag);
 private:
  const std::string getChassisState(uint8_t mode);
  const std::string getGimbalState(uint8_t mode);
  const std::string getShooterState(uint8_t mode);
};

class CapacitorUi : public UiBase<AutoChangeGraph> {
 public:
  explicit CapacitorUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "capacitor") {};
  void add(double data);
  void update(const ros::Time &time, double data);
 private:
  void setConfig(AutoChangeGraph *graph, double data);
};

} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
