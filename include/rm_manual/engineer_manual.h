//
// Created by qiayuan on 5/23/21.
//

#ifndef RM_MANUAL_ENGINEER_MANUAL_H_
#define RM_MANUAL_ENGINEER_MANUAL_H_

#include "rm_manual/chassis_gimbal_manual.h"

namespace rm_manual {
class EngineerManual : public ChassisGimbalManual {
 public:
 private:
  Vel2DCommandSender vel_command_sender_;
};

}
#endif //RM_MANUAL_ENGINEER_MANUAL_H_
