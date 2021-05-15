//
// Created by astro on 2021/5/15.
//

#include "rm_manual/controller_manager.h"
ControllerManager::ControllerManager(ros::NodeHandle &node_handle) : nh_(node_handle) {
  switch_controller_client_ =
      nh_.serviceClient<controller_manager_msgs::SwitchControllerRequest>("/controller_manager/switch_controller");
  list_controllers_client_ =
      nh_.serviceClient<controller_manager_msgs::ListControllersRequest>("/controller_manager/list_controllers");
}
bool ControllerManager::checkControllersLoaded() {
  if (list_controllers_client_.call(list_controller_srv_)) {
    return true;
  } else {
    return false;
  }
}
void ControllerManager::startAllControllers() {
  if (getParam(nh_, "start_controller/robot_state_controller", false)) {
    startRobotStateController();
  }
  if (getParam(nh_, "start_controller/joint_state_controller", false)) {
    startJointStateController();
  }
  if (getParam(nh_, "start_controller/chassis_controller", false)) {
    startChassisController();
  }
  if (getParam(nh_, "start_controller/gimbal_controller", false)) {
    startGimbalController();
  }
  if (getParam(nh_, "start_controller/shooter_controller", false)) {
    startShooterController();
  }
  if (getParam(nh_, "start_controller/imu_sensor_controller", false)) {
    startImuSensorController();
  }
  if (getParam(nh_, "start_controller/engineer_arm_controller", false)) {
    startEngineerArmController();
  }
  if (getParam(nh_, "start_controller/engineer_hand_controller", false)) {
    startEngineerHandController();
  }
  if (getParam(nh_, "start_controller/joint_group_position_controller", false)) {
    startJointGroupPositionController();
  }
}
void ControllerManager::stopAllControllers() {
  if (getParam(nh_, "start_controller/robot_state_controller", false)) {
    stopRobotStateController();
  }
  if (getParam(nh_, "start_controller/joint_state_controller", false)) {
    stopJointStateController();
  }
  if (getParam(nh_, "start_controller/chassis_controller", false)) {
    stopChassisController();
  }
  if (getParam(nh_, "start_controller/gimbal_controller", false)) {
    stopGimbalController();
  }
  if (getParam(nh_, "start_controller/shooter_controller", false)) {
    stopShooterController();
  }
  if (getParam(nh_, "start_controller/imu_sensor_controller", false)) {
    stopImuSensorController();
  }
  if (getParam(nh_, "start_controller/engineer_arm_controller", false)) {
    stopEngineerArmController();
  }
  if (getParam(nh_, "start_controller/engineer_hand_controller", false)) {
    stopEngineerHandController();
  }
  if (getParam(nh_, "start_controller/joint_group_position_controller", false)) {
    stopJointGroupPositionController();
  }
}
void ControllerManager::startRobotStateController() {
  switch_controller_srv_.request.start_controllers.push_back("controllers/robot_state_controller");
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::startJointStateController() {
  switch_controller_srv_.request.start_controllers.push_back("controllers/joint_state_controller");
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::startImuSensorController() {
  switch_controller_srv_.request.start_controllers.push_back("controllers/imu_sensor_controller");
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::startEngineerArmController() {
  switch_controller_srv_.request.start_controllers.push_back("engineer_arm_controller");
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::startEngineerHandController() {
  switch_controller_srv_.request.stop_controllers.push_back("joint_group_position_controller");
  switch_controller_srv_.request.start_controllers.push_back("engineer_hand_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::startJointGroupPositionController() {
  switch_controller_srv_.request.stop_controllers.push_back("engineer_hand_controller");
  switch_controller_srv_.request.start_controllers.push_back("joint_group_position_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::startChassisController() {
  switch_controller_srv_.request.start_controllers.push_back("controllers/chassis_controller");
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::startGimbalController() {
  switch_controller_srv_.request.start_controllers.push_back("controllers/gimbal_controller");
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::startShooterController() {
  switch_controller_srv_.request.start_controllers.push_back("controllers/shooter_controller");
  switch_controller_srv_.request.stop_controllers.clear();
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}

void ControllerManager::stopRobotStateController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("controllers/robot_state_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopJointStateController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("controllers/joint_state_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopImuSensorController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("controllers/imu_sensor_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopEngineerArmController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("engineer_arm_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopEngineerHandController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("engineer_hand_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopJointGroupPositionController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("joint_group_position_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopChassisController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("controllers/chassis_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopGimbalController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("controllers/gimbal_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
void ControllerManager::stopShooterController() {
  switch_controller_srv_.request.start_controllers.clear();
  switch_controller_srv_.request.stop_controllers.push_back("controllers/shooter_controller");
  switch_controller_srv_.request.strictness = switch_controller_srv_.request.BEST_EFFORT;
  switch_controller_srv_.request.start_asap = false;
  switch_controller_srv_.request.timeout = 0.5;
  switch_controller_client_.call(switch_controller_srv_);
}
