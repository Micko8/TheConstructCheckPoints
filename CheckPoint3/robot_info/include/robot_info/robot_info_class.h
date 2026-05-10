#ifndef ROBOT_INFO_CLASS_H
#define ROBOT_INFO_CLASS_H
#include "robot_info/robot_info_class.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>

class RobotInfo {
protected:
  ros::NodeHandle n;

  ros::Publisher info_pub;

  std::string robot_decription, serial_number, ip_address, firmare_version;

  robotinfo_msgs::RobotInfo10Fields msg;
  std::string info_topic;

public:
  RobotInfo(void);
  virtual void publish_data();
};

#endif