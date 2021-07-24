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
  static int id_;
};

class TriggerChangeUi : public UiBase {
 public:
  explicit TriggerChangeUi(ros::NodeHandle &nh, Data &data);
  void update(const std::string &graph_name, const std::string &content);
  void update(const std::string &graph_name, uint8_t mode, bool burst_flag = false, bool option_flag = false);
 private:
  void updateConfig(const std::string &name, Graph *graph, uint8_t mode, bool burst_flag, bool option_flag);
  const std::string getChassisState(uint8_t mode);
  const std::string getTargetState(uint8_t mode);
};

class TimeChangeUi : public UiBase {
 public:
  explicit TimeChangeUi(ros::NodeHandle &nh, Data &data) : UiBase(nh, data, "time_change") {};
  void add() override;
  void update(const std::string &name, const ros::Time &time, double data = 0.);
 private:
  void setCapacitorData(Graph &graph);
  void setEffortData(Graph &graph);
  void setProgressData(Graph &graph, double data);
};

class FixedUi : public UiBase {
 public:
  explicit FixedUi(ros::NodeHandle &nh, Data &data) : UiBase(nh, data, "fixed") {};
  void update();
 private:
  int getShootSpeedIndex();
};

class FlashUi : public UiBase {
 public:
  explicit FlashUi(ros::NodeHandle &nh, Data &data) : UiBase(nh, data, "flash") {};
  void update(const std::string &name, const ros::Time &time, bool state = false);
 private:
  void updateArmorPosition(const std::string &name, Graph *graph);
  uint8_t getArmorId(const std::string &name);
};

} // namespace rm_manual
#endif //RM_MANUAL_REFEREE_UI_H_
