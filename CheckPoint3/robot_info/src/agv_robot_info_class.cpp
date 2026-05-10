#include "robot_info/agv_robot_info_class.h"

AGVRobotInfo::AGVRobotInfo(void) : RobotInfo() {
  maximum_payload = "maximum_payload: 100kg";
}

void AGVRobotInfo::publish_data() {
  RobotInfo::publish_data(); // parent
  msg.data_field_05 = maximum_payload;
  info_pub.publish(msg);
}