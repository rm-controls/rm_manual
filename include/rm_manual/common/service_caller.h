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
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>

namespace rm_manual {
template<class ServiceType>
class ServiceCallerBase {
 public:
  explicit ServiceCallerBase(ros::NodeHandle &nh) {
    if (!nh.getParam("service_name_", service_name_))
      ROS_ERROR("Service name no defined (namespace: %s)", nh.getNamespace().c_str());
    client_ = nh.serviceClient<ServiceType>(service_name_);
  }
  ~ServiceCallerBase() { delete thread_; }
  void callService() {
    thread_ = new std::thread(&ServiceCallerBase::callingThread, &this);
    thread_->detach();
  }
  void callingThread() {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!client_.call(service_))
      ROS_ERROR("Failed to call service %s on %s", typeid(ServiceType).name(), service_name_);
  }
  const ServiceType &getRequest() { return service_; }
  bool isCalling() {
    std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
    return !guard.owns_lock();
  }
 protected:
  std::string service_name_;
  ros::ServiceClient client_;
  ServiceType service_;
  std::thread *thread_{};
  std::mutex mutex_;
};

class LoadControllerService : public ServiceCallerBase<controller_manager_msgs::LoadController> {
  explicit LoadControllerService(ros::NodeHandle &nh)
      : ServiceCallerBase<controller_manager_msgs::LoadController>(nh) {}
};

class SwitchControllerService : public ServiceCallerBase<controller_manager_msgs::SwitchController> {
  explicit SwitchControllerService(ros::NodeHandle &nh) :
      ServiceCallerBase<controller_manager_msgs::SwitchController>(nh) {}
};

}

#endif //RM_MANUAL_COMMON_SERVICE_CALLER_H_
