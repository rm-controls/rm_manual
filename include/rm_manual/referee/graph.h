//
// Created by peter on 2021/7/19.
//

#ifndef RM_MANUAL_INCLUDE_GRAPH_H_
#define RM_MANUAL_INCLUDE_GRAPH_H_

#include <iostream>
#include <rm_common/referee/protocol.h>

namespace rm_manual {
class GraphBase {
 public:
  GraphBase(int id, int x, int y, int width)
      : color_(rm_common::GraphColor::WHITE), operation_(rm_common::GraphOperation::UPDATE) {
    config_.graphic_name_[0] = (uint8_t) (id & 0xff);
    config_.graphic_name_[1] = (uint8_t) ((id >> 8) & 0xff);
    config_.graphic_name_[2] = (uint8_t) ((id >> 16) & 0xff);
    config_.start_x_ = x;
    config_.start_y_ = y;
    config_.width_ = width;
  };
  void setColor(rm_common::GraphColor color) { color_ = color; }
  void setOperation(rm_common::GraphOperation operation) { operation_ = operation; }
 protected:
  rm_common::GraphColor color_;
  rm_common::GraphOperation operation_;
  rm_common::GraphConfig config_;
};

class LineGraph : public GraphBase {
 public:
  LineGraph(int id, int start_x, int start_y, int width, int end_x, int end_y)
      : GraphBase(id, start_x, start_y, width) {
    config_.end_x_ = end_x;
    config_.end_y_ = end_y;
    config_.graphic_type_ = rm_common::GraphType::LINE;
  }
};

class RectangleGraph : public LineGraph {
 public:
  RectangleGraph(int id, int start_x, int start_y, int width, int end_x, int end_y)
      : LineGraph(id, start_x, start_y, width, end_x, end_y) {
    config_.graphic_type_ = rm_common::GraphType::RECTANGLE;
  }
};

class CircleGraph : public GraphBase {
 public:
  CircleGraph(int id, int center_x, int center_y, int width, int radius)
      : GraphBase(id, center_x, center_y, width) {
    config_.radius_ = radius;
    config_.graphic_type_ = rm_common::GraphType::CIRCLE;
  }
};

class EllipseGraph : public GraphBase {
 public:
  EllipseGraph(int id, int center_x, int center_y, int width, int len_x, int len_y)
      : GraphBase(id, center_x, center_y, width) {
    config_.end_x_ = len_x;
    config_.end_y_ = len_y;
    config_.graphic_type_ = rm_common::GraphType::ELLIPSE;
  }
};

class ArcGraph : public EllipseGraph {
 public:
  ArcGraph(int id, int center_x, int center_y, int width, int len_x, int len_y, int start_angle, int end_angle)
      : EllipseGraph(id, center_x, center_y, width, len_x, len_y) {
    config_.start_angle_ = start_angle;
    config_.end_angle_ = end_angle;
    config_.graphic_type_ = rm_common::GraphType::ARC;
  }
};

class StringGraph : public GraphBase {
 public:
  StringGraph(int id, int start_x, int start_y, int width, int char_size, int string_len)
      : GraphBase(id, start_x, start_y, width) {
    config_.start_angle_ = char_size;
    config_.end_angle_ = string_len;
    config_.graphic_type_ = rm_common::GraphType::STRING;
  }
  void setString(const std::string &data) {
    for (int i = 0; i < 30; ++i) {
      if (i < (int) data.size()) string_data_[i] = data[i];
      else string_data_[i] = ' ';
    }
  }
  uint8_t *getString() { return string_data_; }
 private:
  uint8_t string_data_[30]{};
};
}
#endif //RM_MANUAL_INCLUDE_GRAPH_H_
