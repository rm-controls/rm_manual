//
// Created by peter on 2021/7/19.
//

#ifndef RM_MANUAL_INCLUDE_GRAPH_H_
#define RM_MANUAL_INCLUDE_GRAPH_H_

#include "rm_manual/referee/referee.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/referee/protocol.h>

namespace rm_manual {
class GraphBase {
 public:
  explicit GraphBase(const XmlRpc::XmlRpcValue &config, Referee &referee)
      : referee_(referee), content_("") {
    if (config.hasMember("id")) {
      config_.graphic_id_[0] = (uint8_t) ((int) config["id"] & 0xff);
      config_.graphic_id_[1] = (uint8_t) (((int) config["id"] >> 8) & 0xff);
      config_.graphic_id_[2] = (uint8_t) (((int) config["id"] >> 16) & 0xff);
    }
    if (config.hasMember("start_x")) config_.start_x_ = setValue(config["start_x"], start_x_array_);
    if (config.hasMember("start_y")) config_.start_y_ = setValue(config["start_y"], start_y_array_);
    if (config.hasMember("end_x")) config_.end_x_ = setValue(config["end_x"], end_x_array_);
    if (config.hasMember("end_y")) config_.end_y_ = setValue(config["end_y"], end_y_array_);
    if (config.hasMember("type")) config_.graphic_type_ = getType(config["type"]);
    if (config.hasMember("start_angle")) config_.start_angle_ = (int) config["start_angle"];
    if (config.hasMember("end_angle")) config_.end_angle_ = (int) config["end_angle"];
    if (config.hasMember("radius")) config_.radius_ = (int) config["radius"];
    if (config.hasMember("width")) config_.width_ = (int) config["width"];
    if (config.hasMember("color")) config_.color_ = getColor(config["color"]);
    if (config.hasMember("content")) content_ = (std::string) config["content"];
  };
  void setColor(const rm_common::GraphColor &color) { config_.color_ = color; }
  void setContent(const std::string &content) { content_ = content; }
 protected:
  rm_common::GraphColor getColor(const std::string &color) {
    if (color == "main_color") return rm_common::GraphColor::MAIN_COLOR;
    else if (color == "yellow") return rm_common::GraphColor::YELLOW;
    else if (color == "green") return rm_common::GraphColor::GREEN;
    else if (color == "orange") return rm_common::GraphColor::ORANGE;
    else if (color == "purple") return rm_common::GraphColor::PURPLE;
    else if (color == "pink") return rm_common::GraphColor::PINK;
    else if (color == "cyan") return rm_common::GraphColor::CYAN;
    else if (color == "black") return rm_common::GraphColor::BLACK;
    else return rm_common::GraphColor::WHITE;
  }
  rm_common::GraphType getType(const std::string &type) {
    if (type == "rectangle") return rm_common::GraphType::RECTANGLE;
    else if (type == "circle") return rm_common::GraphType::CIRCLE;
    else if (type == "ellipse") return rm_common::GraphType::ELLIPSE;
    else if (type == "arc") return rm_common::GraphType::ARC;
    else if (type == "string") return rm_common::GraphType::STRING;
    else return rm_common::GraphType::LINE;
  }
  void display() { referee_.sendUi(config_, content_); }
  Referee &referee_;
  std::string content_;
  rm_common::GraphConfig config_{};
  int start_x_array_[4]{}, start_y_array_[4]{}, end_x_array_[4]{}, end_y_array_[4]{};
 private:
  int setValue(XmlRpc::XmlRpcValue value, int *storage_array) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
      for (int i = 0; i < 4 && i < (int) value.size(); ++i) storage_array[i] = (int) value[i];
    else storage_array[0] = (int) value;
    return storage_array[0];
  }
};

class UnChangeGraph : public GraphBase {
 public:
  explicit UnChangeGraph(const XmlRpc::XmlRpcValue &config, Referee &referee) : GraphBase(config, referee) {
    config_.operate_type_ = rm_common::GraphOperation::ADD;
    config_.layer_ = 2;
  };
  void add() { display(); }
};

class AutoChangeGraph : public GraphBase {
 public:
  explicit AutoChangeGraph(const XmlRpc::XmlRpcValue &config, Referee &referee)
      : GraphBase(config, referee), last_time_(ros::Time::now()), duration_(ros::Duration(1.)) {
    config_.operate_type_ = rm_common::GraphOperation::UPDATE;
    config_.layer_ = 1;
  };
  void add() {
    config_.operate_type_ = rm_common::GraphOperation::ADD;
    display();
    config_.operate_type_ = rm_common::GraphOperation::UPDATE;
  }
  void update(int data) {
    if (data == last_data_) return;
    updatePosition();
    display();
    last_data_ = data;
  }
  void update(const ros::Time &time, bool state) {
    if (!state || time - last_time_ < duration_) return;
    config_.operate_type_ = config_.operate_type_ == rm_common::GraphOperation::DELETE ? rm_common::GraphOperation::ADD
                                                                                       : rm_common::GraphOperation::DELETE;
    display();
    last_time_ = time;
  }
  void update(const ros::Time &time, double data) {
    if (data == 0. || time - last_time_ < duration_) return;
    display();
    last_time_ = time;
  }
 private:
  void updatePosition() {
    if (level++ > 4) return;
    config_.start_x_ = start_x_array_[level];
    config_.start_y_ = start_y_array_[level];
    config_.end_x_ = end_x_array_[level];
    config_.end_y_ = end_y_array_[level];
  }
  ros::Time last_time_;
  ros::Duration duration_;
  int last_data_{};
  int level{};
};

class ManualChangeGraph : public GraphBase {
 public:
  explicit ManualChangeGraph(const XmlRpc::XmlRpcValue &config, Referee &referee)
      : GraphBase(config, referee), last_state_(0), last_flag_(false) {
    config_.operate_type_ = rm_common::GraphOperation::UPDATE;
    config_.layer_ = 0;
  };
  void add() {
    config_.operate_type_ = rm_common::GraphOperation::ADD;
    display();
    config_.operate_type_ = rm_common::GraphOperation::UPDATE;
  }
  void update(uint8_t state, bool flag) {
    if (state != last_state_ || flag != last_flag_) {
      display();
      last_state_ = state;
      last_flag_ = flag;
    }
  }
 private:
  uint8_t last_state_;
  bool last_flag_;
};

}
#endif //RM_MANUAL_INCLUDE_GRAPH_H_
