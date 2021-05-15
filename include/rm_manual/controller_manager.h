//
// Created by astro on 2021/5/15.
//
#ifndef SRC_RM_SOFTWARE_RM_MANUAL_INCLUDE_RM_MANUAL_CONTROLLER_MANAGER_H_
#define SRC_RM_SOFTWARE_RM_MANUAL_INCLUDE_RM_MANUAL_CONTROLLER_MANAGER_H_
#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <rm_common/ros_utilities.h>
class ControllerManager {
 public:
  ControllerManager(ros::NodeHandle &node_handle);

  bool checkControllersLoaded();
  void startAllControllers();
  void stopAllControllers();
  void startRobotStateController();
  void startJointStateController();
  void startImuSensorController();
  void startEngineerArmController();
  void startEngineerHandController();
  void startJointGroupPositionController();
  void startChassisController();
  void startGimbalController();
  void startShooterController();

  void stopRobotStateController();
  void stopJointStateController();
  void stopImuSensorController();
  void stopEngineerArmController();
  void stopEngineerHandController();
  void stopJointGroupPositionController();
  void stopChassisController();
  void stopGimbalController();
  void stopShooterController();
 private:
  ros::NodeHandle nh_;
  //server client
  ros::ServiceClient switch_controller_client_;
  controller_manager_msgs::SwitchController switch_controller_srv_;

  ros::ServiceClient list_controllers_client_;
  controller_manager_msgs::ListControllers list_controller_srv_;
};

#endif //SRC_RM_SOFTWARE_RM_MANUAL_INCLUDE_RM_MANUAL_CONTROLLER_MANAGER_H_
