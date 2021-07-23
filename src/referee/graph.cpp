//
// Created by peter on 2021/7/22.
//
#include "rm_manual/referee/graph.h"
namespace rm_manual {
Graph::Graph(const XmlRpc::XmlRpcValue &config, Referee &referee) : referee_(referee) {
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
  if (config.hasMember("delay")) delay_ = ros::Duration((double) config["delay"]);
  if (config.hasMember("title")) title_ = (std::string) config["title"];
  if (config.hasMember("content")) content_ = (std::string) config["content"];
  config_.operate_type_ = rm_common::GraphOperation::DELETE;
  last_config_ = config_;
}

void Graph::display() {
  if (config_ == last_config_ && title_ == last_title_ && content_ == last_content_) return;
  if (!title_.empty() && !content_.empty()) config_.end_angle_ = (int) (title_ + content_).size();
  referee_.addUi(config_, title_ + content_, true);
  last_content_ = content_;
  last_title_ = title_;
  last_config_ = config_;
}

void Graph::display(const ros::Time &time) {
  if (time - last_time_ < delay_) return;
  display();
  last_time_ = time;
}

void Graph::display(const ros::Time &time, bool state, bool once) {
  if (once) {
    if (state) {
      last_time_ = time;
      config_.operate_type_ = rm_common::GraphOperation::ADD;
    }
    if (time - last_time_ > delay_) config_.operate_type_ = rm_common::GraphOperation::DELETE;
  } else if (state && time - last_time_ > delay_) {
    config_.operate_type_ =
        config_.operate_type_ == rm_common::GraphOperation::ADD ? rm_common::GraphOperation::DELETE
                                                                : rm_common::GraphOperation::ADD;
    last_time_ = time;
  }
  display();
}

void Graph::updateX(int index) {
  config_.start_x_ = start_x_array_[index];
  config_.end_x_ = end_x_array_[index];
}

void Graph::updateY(int index) {
  config_.start_y_ = start_y_array_[index];
  config_.end_y_ = end_y_array_[index];
}

int Graph::setValue(XmlRpc::XmlRpcValue value, int *array) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    for (int i = 0; i < 4 && i < (int) value.size(); ++i) array[i] = (int) value[i];
  else array[0] = (int) value;
  return array[0];
}

rm_common::GraphColor Graph::getColor(const std::string &color) {
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

rm_common::GraphType Graph::getType(const std::string &type) {
  if (type == "rectangle") return rm_common::GraphType::RECTANGLE;
  else if (type == "circle") return rm_common::GraphType::CIRCLE;
  else if (type == "ellipse") return rm_common::GraphType::ELLIPSE;
  else if (type == "arc") return rm_common::GraphType::ARC;
  else if (type == "string") return rm_common::GraphType::STRING;
  else return rm_common::GraphType::LINE;
}

}
