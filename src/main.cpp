//
// Created by luohx on 7/27/20.
//

#include "rm_manual/common/manual_common.h"

int main(int argc, char **argv) {
  std::string robot;
  Manual *manual_control;
  ros::init(argc, argv, "rm_manual");
  ros::NodeHandle nh("~");
  robot = getParam(nh, "robot_type", (std::string) "error");
  if (robot == "standard" || robot == "hero")
    manual_control = new Manual(nh);
  else {
    ROS_ERROR("no robot type ");
    return 0;
  }
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    manual_control->run();
    loop_rate.sleep();
  }
  return 0;
}