//
// Created by qiayuan on 7/17/21.
//

#pragma once

#include <ros/ros.h>
#include <utility>

namespace rm_manual
{
class InputEvent
{
public:
  InputEvent() : last_state_(false)
  {
  }
  void setRising(boost::function<void()> handler)
  {
    rising_handler_ = std::move(handler);
  }
  void setFalling(boost::function<void()> handler)
  {
    falling_handler_ = std::move(handler);
  }
  void setRisingDelay(boost::function<void()> handler, ros::Duration delay)
  {
    rising_handler_ = std::move(handler);
    delay_time_ = delay;
  }
  void setFallingDelay(boost::function<void()> handler, ros::Duration delay)
  {
    rising_handler_ = std::move(handler);
    delay_time_ = delay;
  }
  void setActiveHigh(boost::function<void(ros::Duration)> handler)
  {
    active_high_handler_ = std::move(handler);
  }
  void setActiveLow(boost::function<void(ros::Duration)> handler)
  {
    active_low_handler_ = std::move(handler);
  }
  void setEdge(boost::function<void()> rising_handler, boost::function<void()> falling_handler)
  {
    rising_handler_ = std::move(rising_handler);
    falling_handler_ = std::move(falling_handler);
  }
  void setActive(boost::function<void(ros::Duration)> high_handler, boost::function<void(ros::Duration)> low_handler)
  {
    active_high_handler_ = std::move(high_handler);
    active_low_handler_ = std::move(low_handler);
  }
  void startDelay()
  {
    start_time_ = ros::Time::now();
    delay_switch_ = true;
  }
  void update(bool state)
  {
    if (state != last_state_)
    {
      if (!delay_time_.toSec() && state && rising_handler_)
        rising_handler_();
      else if (!delay_time_.toSec() && !state && falling_handler_)
        falling_handler_();
      else if (delay_time_.toSec() && ((!state && falling_handler_) || (state && rising_handler_)))
        startDelay();

      last_state_ = state;
      last_change_ = ros::Time::now();
    }
    if (delay_switch_ && (ros::Time::now() - start_time_ > delay_time_))
    {
      if (rising_handler_)
        rising_handler_();
      else if (falling_handler_)
        falling_handler_();
      delay_switch_ = false;
    }
    if (state && active_high_handler_)
      active_high_handler_(ros::Time::now() - last_change_);
    if (!state && active_low_handler_)
      active_low_handler_(ros::Time::now() - last_change_);
  }

private:
  bool last_state_, delay_switch_;
  ros::Time last_change_, start_time_;
  ros::Duration delay_time_ = ros::Duration(0.);
  boost::function<void(ros::Duration)> active_high_handler_, active_low_handler_;
  boost::function<void()> rising_handler_, falling_handler_;
};

}  // namespace rm_manual
