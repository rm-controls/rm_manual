//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_COMMON_SERVICE_CALLER_H_
#define RM_MANUAL_COMMON_SERVICE_CALLER_H_

#include <chrono>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <ros/service.h>
#include <controller_manager_msgs/SwitchController.h>
#include <control_msgs/QueryCalibrationState.h>

namespace rm_manual {
template<class ServiceType>
class ServiceCallerBase {
 public:
  explicit ServiceCallerBase(ros::NodeHandle &nh) {
    if (!nh.getParam("service_name", service_name_))
      ROS_ERROR("Service name no defined (namespace: %s)", nh.getNamespace().c_str());
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }
  ~ServiceCallerBase() { delete thread_; }
  void callService() {
    if (isCalling()) {
      ROS_INFO("is calling");
      return;
    }
    thread_ = new std::thread(&ServiceCallerBase::callingThread, this);
    thread_->detach();
  }
  ServiceType &getService() { return service_; }
  bool isCalling() {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    return !guard.owns_lock();
  }
 protected:
  void callingThread() {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!client_.call(service_))
      ROS_ERROR("Failed to call service %s on %s", typeid(ServiceType).name(), service_name_.c_str());
  }

  std::string service_name_;
  ros::ServiceClient client_;
  ServiceType service_;
  std::thread *thread_{};
  std::mutex mutex_;
};

class SwitchControllerService : public ServiceCallerBase<controller_manager_msgs::SwitchController> {
 public:
  explicit SwitchControllerService(ros::NodeHandle &nh) :
      ServiceCallerBase<controller_manager_msgs::SwitchController>(nh) {
    XmlRpc::XmlRpcValue controllers;
    if (nh.getParam("start_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
        start_controllers_.push_back(controllers[i]);
    if (nh.getParam("stop_controllers", controllers))
      for (int i = 0; i < controllers.size(); ++i)
        stop_controllers_.push_back(controllers[i]);
    if (start_controllers_.empty() && stop_controllers_.empty())
      ROS_ERROR("No start/stop controllers specified (namespace: %s)", nh.getNamespace().c_str());
    service_.request.strictness = service_.request.BEST_EFFORT;
  }
  void startControllersOnly() {
    service_.request.start_controllers = start_controllers_;
    service_.request.stop_controllers.clear();
  }
  void stopControllersOnly() {
    service_.request.stop_controllers = stop_controllers_;
    service_.request.start_controllers.clear();
  }
  void switchControllers() {
    service_.request.start_controllers = start_controllers_;
    service_.request.stop_controllers = stop_controllers_;
  };
  void flipControllers() {
    service_.request.start_controllers = stop_controllers_;
    service_.request.stop_controllers = start_controllers_;
  };
  bool getOk() {
    if (isCalling()) return false;
    return service_.response.ok;
  }
 private:
  std::vector<std::string> start_controllers_, stop_controllers_;
};

class QueryCalibrationService : public ServiceCallerBase<control_msgs::QueryCalibrationState> {
 public:
  explicit QueryCalibrationService(ros::NodeHandle &nh) : ServiceCallerBase<control_msgs::QueryCalibrationState>(nh) {}
  bool getIsCalibrated() {
    if (isCalling()) return false;
    return service_.response.is_calibrated;
  }
};

}

#endif //RM_MANUAL_COMMON_SERVICE_CALLER_H_
