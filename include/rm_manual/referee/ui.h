//
// Created by peter on 2021/5/24.
//

#ifndef RM_MANUAL_REFEREE_UI_H_
#define RM_MANUAL_REFEREE_UI_H_

#include "rm_manual/referee/graph.h"
#include "rm_manual/common/data.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/StatusChangeRequest.h>

namespace rm_manual {
class UiBase {
 public:
  explicit UiBase(ros::NodeHandle &nh, Data &data, const std::string &ui_type);
  virtual void add();
 protected:
  Data &data_;
  std::map<std::string, Graph *> graph_vector_;
};

class StateUi : public UiBase {
 public:
  explicit StateUi(ros::NodeHandle &nh, Data &data) : UiBase(nh, data, "state") {
    for (auto graph:graph_vector_) updateConfig(graph.first, graph.second, 0, false, false);
  }
  void update(const std::string &graph_name, uint8_t mode, bool burst_flag, bool color_flag = false);
 private:
  void updateConfig(const std::string &name, Graph *graph, uint8_t mode, bool burst_flag, bool color_flag);
  const std::string getChassisState(uint8_t mode);
  const std::string getTargetState(uint8_t mode);
};

class AimUi : public UiBase {
 public:
  explicit AimUi(ros::NodeHandle &nh, Data &data) : UiBase(nh, data, "aim") {};
  void update();
 private:
  void updateConfig(Graph *graph, int level);
};

class ArmorUi : public UiBase {
 public:
  explicit ArmorUi(ros::NodeHandle &nh, Data &data) : UiBase(nh, data, "armor") {};
  void update(const ros::Time &time);
 private:
  void updateConfig(const std::string &name, Graph *graph);
  uint8_t getArmorId(const std::string &name);
};

class WarningUi : public UiBase {
 public:
  explicit WarningUi(ros::NodeHandle &nh, Data &data) : UiBase(nh, data, "warning") {};
  void update(const std::string &name, const ros::Time &time, bool state);
};

class DataUi : public UiBase {
 public:
  explicit DataUi(ros::NodeHandle &nh, Data &data) : UiBase(nh, data, "capacitor") {};
  void add() override;
  void update(const std::string &name, const ros::Time &time);
 private:
  void setCapacitorData(Graph &graph);
  void setEffortData(Graph &graph);
};

} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
