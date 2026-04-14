#include "robot_info/agv_robot_info_class.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "agv_robot_info");
  AGVRobotInfo object;

  ros::Rate rate(1);
  while (ros::ok()) {
    object.publish_data();
    ros::spinOnce();
    rate.sleep();
  }
}