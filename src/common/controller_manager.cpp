//
// Created by astro on 2021/5/15.
//

#include "rm_manual/common/controller_manager.h"
namespace rm_manual {

ControllerManager::ControllerManager(ros::NodeHandle &nh) {
  ros::NodeHandle ctrl_handle(nh, "controller_manager");
  switch_controller_client_ =
      ctrl_handle.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
  load_controllers_client_ =
      ctrl_handle.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
  XmlRpc::XmlRpcValue controllers;
  if (!ctrl_handle.getParam("information_controllers", controllers))
    ROS_INFO("No information controllers defined");
  ROS_ASSERT(controllers.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < controllers.size(); ++i)
    information_controllers_.push_back(controllers[i]);
  if (!ctrl_handle.getParam("movement_controllers", controllers))
    ROS_INFO("No movement controllers defined");
  for (int i = 0; i < controllers.size(); ++i)
    movement_controllers_.push_back(controllers[i]);
  if (!ctrl_handle.getParam("calibration_controllers", controllers))
    ROS_INFO("No calibration controllers defined");
  for (int i = 0; i < controllers.size(); ++i)
    calibration_controllers_.push_back(controllers[i]);

  ros::NodeHandle cali_handle(nh, "calibration_manager");
  XmlRpc::XmlRpcValue rpc_value;
  if (!nh.getParam("calibration_manager", rpc_value))
    ROS_INFO("No trigger calibration controllers defined");
  ROS_ASSERT(rpc_value.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = rpc_value.begin(); it != rpc_value.end(); ++it) {
    std::string value = it->first;
    ros::NodeHandle switch_nh(cali_handle, value + "/switch");
    ros::NodeHandle query_nh(cali_handle, value + "/query");
    calibration_services_.push_back(CalibrationService{
        .switch_services_ = new SwitchControllerService(switch_nh),
        .query_services_ = new QueryCalibrationService(query_nh)});
  }
  last_query_ = ros::Time::now();
  reset();
}

bool ControllerManager::loadControllers(const std::vector<std::string> &controllers) {
  load_controllers_client_.waitForExistence();
  controller_manager_msgs::LoadController load_controller;
  bool is_success = true;
  for (auto &controller : controllers) {
    load_controller.request.name = controller;
    load_controllers_client_.call(load_controller);
    if (load_controller.response.ok)
      ROS_INFO("loaded %s", controller.c_str());
    else {
      ROS_ERROR("fail to load %s", controller.c_str());
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
void ControllerManager::checkCalibrate(const ros::Time &time) {
  if (isCalibrated())
    return;
  if (calibration_itr_->switch_services_->getOk()) {
    if (calibration_itr_->query_services_->getIsCalibrated()) {
      calibration_itr_->switch_services_->flipControllers();
      calibration_itr_->switch_services_->callService();
      calibration_itr_++;
      ROS_INFO("calibration finish");
    } else if ((time - last_query_).toSec() > 1.) {
      last_query_ = time;
      calibration_itr_->query_services_->callService();
    }
  } else {
    calibration_itr_->switch_services_->switchControllers();
    calibration_itr_->switch_services_->callService();
  }
}
void ControllerManager::reset() {
  calibration_itr_ = calibration_services_.begin();
  for (auto service:calibration_services_) {
    service.switch_services_->getService().response.ok = false;
    service.query_services_->getService().response.is_calibrated = false;
  }
}
}