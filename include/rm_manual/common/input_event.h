//
// Created by qiayuan on 7/17/21.
//

#ifndef RM_MANUAL_INCLUDE_INPUT_EVENT_H_
#define RM_MANUAL_INCLUDE_INPUT_EVENT_H_
#include <ros/ros.h>
#include <utility>

namespace rm_manual {
class InputEventBase {
 public:
  explicit InputEventBase(boost::function<void(ros::Duration)> handler) :
      last_state_(false), last_change_(ros::Time::now()), handler_(std::move(handler)) {}
  void callHandle() { handler_(ros::Time::now() - last_change_); }
  virtual void update(bool key_state) {
    if (key_state != last_state_) {
      last_state_ = key_state;
      last_change_ = ros::Time::now();
    }
  }
 protected:
  bool last_state_;
  ros::Time last_change_;
  boost::function<void(ros::Duration duration)> handler_;
};

class RisingInputEvent : public InputEventBase {
 public:
  using InputEventBase::InputEventBase;
  void update(bool key_state) override {
    if (key_state && !last_state_) callHandle();
    InputEventBase::update(key_state);
  }
};

class FallingInputEvent : public InputEventBase {
 public:
  using InputEventBase::InputEventBase;
  void update(bool key_state) override {
    if (!key_state && last_state_) callHandle();
    InputEventBase::update(key_state);
  }
};

class ActiveHighInputEvent : public InputEventBase {
 public:
  using InputEventBase::InputEventBase;
  void update(bool key_state) override {
    if (key_state) callHandle();
    InputEventBase::update(key_state);
  }
};

}
#endif //RM_MANUAL_INCLUDE_INPUT_EVENT_H_
