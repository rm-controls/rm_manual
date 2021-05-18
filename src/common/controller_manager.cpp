//
// Created by astro on 2021/5/15.
//

#include "rm_manual/common/controller_manager.h"

ControllerManager::ControllerManager(ros::NodeHandle &nh) {
  switch_controller_client_ =
      nh.serviceClient<controller_manager_msgs::SwitchControllerRequest>("/controller_manager/switch_controller");
  load_controllers_client_ =
      nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
  list_controllers_client_ =
      nh.serviceClient<controller_manager_msgs::ListControllersRequest>("/controller_manager/list_controllers");
  XmlRpc::XmlRpcValue controllers;
  if (nh.getParam("information_controllers", controllers))
    ROS_INFO("No information controllers defined");
  for (int i = 0; i < controllers.size(); ++i)
    information_controllers_.push_back(controllers[0][i]);
  if (nh.getParam("information_controllers", controllers))
    ROS_INFO("No movement controllers defined");
  for (int i = 0; i < controllers.size(); ++i)
    movement_controllers_.push_back(controllers[0][i]);
}

bool ControllerManager::loadControllers(const std::vector<std::string> &controllers) {
  if (load_controllers_client_.waitForExistence(ros::Duration(1.0)))
    return false;
  controller_manager_msgs::LoadController load_controller;
  bool is_success = true;
  for (auto &controller : controllers) {
    load_controller.request.name = controller;
    if (load_controllers_client_.call(load_controller))
      ROS_INFO("loaded %s", controller.c_str());
    else {
      ROS_INFO("fail to load %s", controller.c_str());
      is_success = false;
    }
  }
  return is_success;
}

bool ControllerManager::switchController(const std::vector<std::string> &start, const std::vector<std::string> &stop) {
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.strictness = switch_controller.request.BEST_EFFORT;
  switch_controller.request.start_asap = true;
  switch_controller.request.timeout = 0.1;
  for (auto &controller : start)
    switch_controller.request.start_controllers.push_back(controller);
  for (auto &controller : stop)
    switch_controller.request.stop_controllers.push_back(controller);
  return switch_controller_client_.call(switch_controller);
}