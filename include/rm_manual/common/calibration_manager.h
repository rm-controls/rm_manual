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
    ROS_ASSERT(rpc_value.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (int i = 0; i < rpc_value.size(); ++i) {
      std::string value = rpc_value[i];
      ros::NodeHandle switch_nh(nh, value + "/switch");
      ros::NodeHandle query_nh(nh, value + "/query");
      calibration_services_.push_back(CalibrationService{
          .switch_services_ = new SwitchControllerService(switch_nh),
          .query_services_ = new QueryCalibrationService(query_nh)});
    }
    reset();
  }
  void calibrate() {
    if (isCalibrated())
      return;
    if (calibration_itr_->switch_services_->getOk() && calibration_itr_->query_services_->getIsCalibrated())
      calibration_itr_++;
    else {
      calibration_itr_->switch_services_->callService();
      calibration_itr_->query_services_->callService();
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

#endif //RM_MANUAL_CALIBRATION_MANAGER_H_
