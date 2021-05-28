//
// Created by astro on 2021/5/15.
//
#ifndef RM_MANUAL_INCLUDE_RM_MANUAL_CONTROLLER_MANAGER_H_
#define RM_MANUAL_INCLUDE_RM_MANUAL_CONTROLLER_MANAGER_H_
#include "service_caller.h"

#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
namespace rm_manual {
class ControllerLoader {
 public:
  explicit ControllerLoader(ros::NodeHandle &nh) {
    load_controllers_client_ =
        nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
    XmlRpc::XmlRpcValue controllers;
    if (!nh.getParam("controllers_list", controllers))
      ROS_INFO("No controllers defined");
    for (int i = 0; i < controllers.size(); ++i)
      controllers_list_.push_back(controllers[i]);
  }
  bool loadControllers() {
    ROS_INFO("Waiting for load_controller service...");
    load_controllers_client_.waitForExistence();
    bool is_success = true;
    for (auto &controller : controllers_list_) {
      controller_manager_msgs::LoadController load_controller;
      load_controller.request.name = controller;
      load_controllers_client_.call(load_controller);
      if (load_controller.response.ok)
        ROS_INFO("Loaded %s", controller.c_str());
      else {
        ROS_ERROR("Fail to load %s", controller.c_str());
        is_success = false;
      }
    }
    return is_success;
  }
 private:
  ros::ServiceClient load_controllers_client_;
  std::vector<std::string> controllers_list_;
};

}
#endif //RM_MANUAL_INCLUDE_RM_MANUAL_CONTROLLER_MANAGER_H_
