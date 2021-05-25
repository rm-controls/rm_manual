//
// Created by qiayuan on 5/23/21.
//

#ifndef RM_MANUAL_CALIBRATION_MANAGER_H_
#define RM_MANUAL_CALIBRATION_MANAGER_H_
#include "rm_manual/common/service_caller.h"
namespace rm_manual {
struct CalibrationService {
  SwitchControllerService *switch_services_;
  QueryCalibrationService *query_services_;
};

class CalibrationManager {
 public:
  explicit CalibrationManager(ros::NodeHandle &nh) {
    XmlRpc::XmlRpcValue rpc_value;
    ros::NodeHandle cali_handle(nh, "calibration_manager");
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
    reset();
  }
  void calibrate() {
    if (isCalibrated())
      return;
    if (calibration_itr_->switch_services_->getOk() && calibration_itr_->query_services_->getIsCalibrated()) {
      calibration_itr_++;
    }
  }
  bool isCalibrated() { return calibration_itr_ == calibration_services_.end(); }
  void reset() {
    calibration_itr_ = calibration_services_.begin();
    for (auto service:calibration_services_) {
      service.switch_services_->getService().response.ok = false;
      service.query_services_->getService().response.is_calibrated = false;
    }
  }
 private:
  std::vector<CalibrationService> calibration_services_;
  std::vector<CalibrationService>::iterator calibration_itr_;
};
}

#endif // RM_MANUAL_CALIBRATION_MANAGER_H_
