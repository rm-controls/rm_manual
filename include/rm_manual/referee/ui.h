//
// Created by peter on 2021/5/24.
//

#ifndef RM_MANUAL_REFEREE_UI_H_
#define RM_MANUAL_REFEREE_UI_H_

#include "rm_manual/referee/graph.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/StatusChangeRequest.h>

namespace rm_manual {
class UiBase {
 public:
  explicit UiBase(ros::NodeHandle &nh, Referee &referee, const std::string &ui_type);
  virtual void add();
 protected:
  Referee &referee_;
  std::map<std::string, Graph *> graph_vector_;
};

class StateUi : public UiBase {
 public:
  explicit StateUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "state") {
    for (auto graph:graph_vector_) updateConfig(graph.first, graph.second, 0, false);
  }
  void update(const std::string &graph_name, uint8_t mode, bool flag = false);
 private:
  void updateConfig(const std::string &name, Graph *graph, uint8_t mode, bool flag);
  const std::string getChassisState(uint8_t mode);
  const std::string getGimbalState(uint8_t mode);
  const std::string getShooterState(uint8_t mode);
  const std::string getTargetState(uint8_t mode);
};

class AimUi : public UiBase {
 public:
  explicit AimUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "aim") {};
  void update();
 private:
  void updatePosition(Graph *graph, int level);
};

class CapacitorUi : public UiBase {
 public:
  explicit CapacitorUi(ros::NodeHandle &nh, Referee &referee) : UiBase(nh, referee, "capacitor") {};
  void add() override;
  void update(const ros::Time &time);
 private:
  void setConfig(Graph *graph, double data);
};

} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
