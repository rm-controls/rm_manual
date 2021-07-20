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
  explicit GraphBase(const XmlRpc::XmlRpcValue &config, Referee &referee) : referee_(referee) {
    config_.graphic_id_[0] = (uint8_t) ((int) config["id"] & 0xff);
    config_.graphic_id_[1] = (uint8_t) (((int) config["id"] >> 8) & 0xff);
    config_.graphic_id_[2] = (uint8_t) (((int) config["id"] >> 16) & 0xff);
    config_.start_x_ = (int) config["start_x"];
    config_.start_y_ = (int) config["start_y"];
    config_.end_x_ = (int) config["end_x"];
    config_.end_y_ = (int) config["end_y"];
    config_.start_angle_ = (int) config["start_angle"];
    config_.end_angle_ = (int) config["end_angle"];
    config_.radius_ = (int) config["radius"];
    config_.width_ = (int) config["width"];
    config_.color_ = getColor(config["color"]);
    config_.graphic_type_ = getType(config["type"]);
    content_ = (std::string) config["content"];
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
  rm_common::GraphConfig config_;
  std::string name_;
  std::string content_;
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
  void update(const ros::Time &time, bool state) {
    if (state && time - last_time_ > duration_) {
      if (config_.operate_type_ == rm_common::GraphOperation::DELETE)
        config_.operate_type_ = rm_common::GraphOperation::ADD;
      else config_.operate_type_ = rm_common::GraphOperation::DELETE;
      display();
      last_time_ = time;
    }
  }
  void update(const ros::Time &time, double data) {
    if (data != 0. && time - last_time_ > duration_) {
      display();
      last_time_ = time;
    }
  }
 private:
  ros::Time last_time_;
  ros::Duration duration_;
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
