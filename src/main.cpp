//
// Created by luohx on 7/27/20.
//

#include "rm_manual/manual_common.h"
int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "rm_manual");
  ros::NodeHandle nh("~");
  robot = getParam(nh, "robot_type", (std::string) "error");
  //Fsm<float> *control_fsm;
  if (robot == "standard") {
    ROS_INFO("Running standard robot.");
  }else {
    ROS_ERROR("No robot type load");
    return 0;
  }
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    //control_fsm->run();
  }
  return 0;
}