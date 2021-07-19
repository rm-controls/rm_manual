//
// Created by peter on 2021/7/19.
//

#ifndef RM_MANUAL_INCLUDE_GRAPH_H_
#define RM_MANUAL_INCLUDE_GRAPH_H_

#include <rm_common/ros_utilities.h>
#include <rm_common/referee/protocol.h>

namespace rm_manual {
class GraphBase {
 public:
  explicit GraphBase(const XmlRpc::XmlRpcValue &config) {
    try {
      config_.graphic_name_[0] = (uint8_t) ((int) config["id"] & 0xff);
      config_.graphic_name_[1] = (uint8_t) (((int) config["id"] >> 8) & 0xff);
      config_.graphic_name_[2] = (uint8_t) (((int) config["id"] >> 16) & 0xff);
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
    } catch (XmlRpc::XmlRpcException &e) {
      ROS_ERROR("Wrong ui parameter type: %s", e.getMessage().c_str());
    }
  };
  const rm_common::GraphConfig &getConfig() { return config_; }
  const std::string &getContent() { return content_; }
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
  rm_common::GraphConfig config_;
  std::string content_;
};

class UnChangeGraph : public GraphBase {
 public:
  explicit UnChangeGraph(const XmlRpc::XmlRpcValue &config) : GraphBase(config) {
    config_.operate_type_ = rm_common::GraphOperation::ADD;
    config_.layer_ = 2;
  };
};

class AutoChangeGraph : public GraphBase {
 public:
  explicit AutoChangeGraph(const XmlRpc::XmlRpcValue &config) : GraphBase(config) {
    config_.operate_type_ = rm_common::GraphOperation::UPDATE;
    config_.layer_ = 1;
  };
  void setContent(const std::string &content) { content_ = content; }
  void setColor(const rm_common::GraphColor &color) { config_.color_ = color; }
  void setOperation(const rm_common::GraphOperation &operation) { config_.operate_type_ = operation; }
};

class ManualChangeGraph : public GraphBase {
 public:
  explicit ManualChangeGraph(const XmlRpc::XmlRpcValue &config) : GraphBase(config) {
    config_.operate_type_ = rm_common::GraphOperation::UPDATE;
    config_.layer_ = 0;
  };
  void setContent(const std::string &content) { content_ = content; }
  void setColor(const rm_common::GraphColor &color) { config_.color_ = color; }
  void setOperation(const rm_common::GraphOperation &operation) { config_.operate_type_ = operation; }
};

}
#endif //RM_MANUAL_INCLUDE_GRAPH_H_
