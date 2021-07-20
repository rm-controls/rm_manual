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
  explicit UiBase(ros::NodeHandle &nh, Referee &referee, const std::string &ui_type);
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
  explicit WarningUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "title") {};
  void update(const ros::Time &time, const std::string &graph_name, bool state);
};

class CapacitorUi : public UiBase<AutoChangeGraph> {
 public:
  explicit CapacitorUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "title") {};
  void add(double data);
  void update(const ros::Time &time, double data);
 private:
  void setConfig(AutoChangeGraph &config, double data);
};

class ControllersUi : public UiBase<ManualChangeGraph> {
 public:
  explicit ControllersUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "title") {};
  void add() { for (auto graph:graph_vector_) graph.second->add(); }
  void update(const std::string &graph_name, uint8_t mode, bool flag);
 protected:
  void updateConfig(const std::string &name, ManualChangeGraph *graph, uint8_t mode, bool flag);
 private:
  const std::string getChassisState(uint8_t mode);
  const std::string getGimbalState(uint8_t mode);
  const std::string getShooterState(uint8_t mode);
};
} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
