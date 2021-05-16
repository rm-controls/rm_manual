//
// Created by luohx on 7/27/20.
//

#include "rm_manual/manual_common.h"

int main(int argc, char **argv) {
  std::string robot;
  Manual *manual_control;
  ros::init(argc, argv, "rm_manual");
  ros::NodeHandle nh("~");
  robot = getParam(nh, "robot_type", (std::string) "error");
  if (robot == "standard")
    manual_control = new Manual(nh);
  else if (robot == "hero")
    manual_control = new Manual(nh);
  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    manual_control->run();
  }
  return 0;
}