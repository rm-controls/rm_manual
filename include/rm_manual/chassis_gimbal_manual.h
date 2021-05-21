//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSISGIMBALMANUAL_H_
#define RM_MANUAL_CHASSISGIMBALMANUAL_H_

#include "rm_manual/common/manual_base.h"
namespace rm_manual {
class ChassisGimbalManual : public ManualBase {
 public:
  explicit ChassisGimbalManual(ros::NodeHandle &nh) : ManualBase(nh) {
    ros::NodeHandle chassis_nh(nh, "chassis");
    chassis_command_sender_ = new ChassisCommandSender(chassis_nh);
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_command_sender_ = new GimbalCommandSender(gimbal_nh, *data_.referee_);
  }
 protected:
  virtual void rightSwitchMid() override;
  virtual void rightSwitchUp() override;
  void wPress() override {}
  void aPress() override {}
  void sPress() override {}
  void dPress() override {}
  ChassisCommandSender *chassis_command_sender_;
  GimbalCommandSender *gimbal_command_sender_;
};
}

#endif //RM_MANUAL_CHASSISGIMBALMANUAL_H_
