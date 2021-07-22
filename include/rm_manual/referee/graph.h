//
// Created by peter on 2021/7/19.
//

#ifndef RM_MANUAL_INCLUDE_GRAPH_H_
#define RM_MANUAL_INCLUDE_GRAPH_H_

#include "rm_manual/referee/referee.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/referee/protocol.h>

namespace rm_manual {
class Graph {
 public:
  explicit Graph(const XmlRpc::XmlRpcValue &config, Referee &referee);
  void display();
  void display(const ros::Time &time);
  void display(const ros::Time &time, bool state);
  void updateX(int index);
  void updateY(int index);
  void setOperation(const rm_common::GraphOperation &operation) { config_.operate_type_ = operation; }
  void setColor(const rm_common::GraphColor &color) { config_.color_ = color; }
  void setContent(const std::string &content) { content_ = content; }
  void setStartX(int start_x) { config_.start_x_ = start_x; }
  void setStartY(int start_y) { config_.start_y_ = start_y; }
  const std::string getTitle() { return title_; }
 private:
  int setValue(XmlRpc::XmlRpcValue value, int *array);
  rm_common::GraphColor getColor(const std::string &color);
  rm_common::GraphType getType(const std::string &type);
  Referee &referee_;
  rm_common::GraphConfig config_{}, last_config_{};
  std::string title_{}, content_{}, last_title_{}, last_content_{};
  ros::Time last_time_ = ros::Time::now();
  ros::Duration delay_ = ros::Duration(0.);
  int start_x_array_[4]{}, start_y_array_[4]{}, end_x_array_[4]{}, end_y_array_[4]{};
};

}
#endif //RM_MANUAL_INCLUDE_GRAPH_H_
