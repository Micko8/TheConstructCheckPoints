#include "robot_info/robot_info_class.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_info");
  RobotInfo object;
  ros::Rate rate(1); // 1 Hz
  while (ros::ok()) {
    object.publish_data();
    ros::spinOnce();
    rate.sleep();
  }
}