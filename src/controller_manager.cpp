//
// Created by astro on 2021/5/15.
//

#include "rm_manual/controller_manager.h"
ControllerManager::ControllerManager(ros::NodeHandle &node_handle) : nh_(node_handle) {
  switch_controller_client_ =
      nh_.serviceClient<controller_manager_msgs::SwitchControllerRequest>("/controller_manager/switch_controller");
  load_controllers_client_ =
      nh_.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
  list_controllers_client_ =
      nh_.serviceClient<controller_manager_msgs::ListControllersRequest>("/controller_manager/list_controllers");
  base_controllers_ = new XmlRpc::XmlRpcValue;
  module_controllers_ = new XmlRpc::XmlRpcValue;
  nh_.getParam("base_controllers", *base_controllers_);
  nh_.getParam("module_controllers", *module_controllers_);
  for (int i = 0; i < base_controllers_->size(); ++i) {
    base_controllers_vector_.push_back(base_controllers_[0][i]);
  }
  for (int i = 0; i < module_controllers_->size(); ++i) {
    module_controllers_vector_.push_back(module_controllers_[0][i]);
  }
}
void ControllerManager::InitControllers() {
  load_controllers_client_.waitForExistence(ros::Duration(1.5));
  for (auto &controller : base_controllers_vector_) {
    load_controller_srv_.request.name.clear();
    load_controller_srv_.request.name.append(controller);
    switch_controller_srv_.request.start_controllers.push_back(controller);
    if (load_controllers_client_.call(load_controller_srv_))
      ROS_INFO("loaded %s", controller.c_str());
    else
      ROS_INFO("fail to load %s", controller.c_str());
  }
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  if (switch_controller_client_.call(switch_controller_srv_)) {
    ROS_INFO("start base controllers");
  } else {
    ROS_INFO("can not start base controllers");
  }
  for (auto &controller : module_controllers_vector_) {
    load_controller_srv_.request.name.clear();
    load_controller_srv_.request.name.append(controller);
    if (load_controllers_client_.call(load_controller_srv_))
      ROS_INFO("loaded %s", controller.c_str());
    else
      ROS_INFO("fail to load %s", controller.c_str());
  }
}
void ControllerManager::startAllControllers() {
  for (auto &controller : module_controllers_vector_) {
    switch_controller_srv_.request.start_controllers.push_back(controller);
  }
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  if (switch_controller_client_.call(switch_controller_srv_)) {
    ROS_INFO("start controllers");
  } else {
    ROS_INFO("can not start controllers");
  }
}
void ControllerManager::stopAllControllers() {
  switch_controller_srv_.request.start_controllers.clear();
  for (auto &controller : module_controllers_vector_) {
    switch_controller_srv_.request.stop_controllers.push_back(controller);
  }
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  if (switch_controller_client_.call(switch_controller_srv_)) {
    ROS_INFO("stop controllers");
  } else {
    ROS_INFO("can not stop controllers");
  }
}

