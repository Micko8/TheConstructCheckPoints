#include "robot_info/robot_info_class.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>

RobotInfo::RobotInfo(void) {
  n = ros::NodeHandle("~");
  info_topic = "/robot_info";
  info_pub = n.advertise<robotinfo_msgs::RobotInfo10Fields>(
      n.resolveName(info_topic), 1);

  robot_decription = "robot_description: Cooper100";
  serial_number = "serial_number: 567A359";
  ip_address = "ip_address: 169.254.5.180";
  firmare_version = "firmware_version: 3.5.8";
  msg.data_field_01 = robot_decription;
  msg.data_field_02 = serial_number;
  msg.data_field_03 = ip_address;
  msg.data_field_04 = firmare_version;

  ROS_INFO("Initializing node");
  usleep(2000000);
}

void RobotInfo::publish_data() { info_pub.publish(msg); }