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
  controllers_ = new XmlRpc::XmlRpcValue;
  nh_.getParam("load_controllers", *controllers_);
}
void ControllerManager::loadAllControllers() {
  for (int i = 0; i < controllers_->size(); ++i) {

    load_controller_srv_.request.name.clear();
    load_controller_srv_.request.name.append(controllers_[0][i]);
    while (!load_controllers_client_.call(load_controller_srv_)) {
      ros::Duration(0.05).sleep();//load controller need some time
    }
    std::cout << "loaded " << controllers_[0][i] << std::endl;
  }
}
void ControllerManager::startAllControllers() {
  for (int i = 0; i < controllers_->size(); ++i) {
    switch_controller_srv_.request.start_controllers.push_back(controllers_[0][i]);
    std::cout << "statrt" << (std::string) controllers_[0][i] << std::endl;
  }
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopAllControllers() {
  switch_controller_srv_.request.start_controllers.clear();
  for (int i = 0; i < controllers_->size(); ++i) {
    switch_controller_srv_.request.stop_controllers.push_back(controllers_[0][i]);
  }
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}

