#include "robot_info/agv_robot_info_class.h"
#include <vector>

AGVRobotInfo::AGVRobotInfo(void) : RobotInfo() {
  maximum_payload = "maximum_payload: 100kg";
  HydraulicSystemMonitor hydroMonitorObject;
}

void AGVRobotInfo::publish_data() {
  RobotInfo::publish_data();
  msg.data_field_05 = maximum_payload;
  std::vector<std::string> hydro = hydroMonitorObject.getAll();
  msg.data_field_06 = hydro[0];
  msg.data_field_07 = hydro[1];
  msg.data_field_08 = hydro[2];
  info_pub.publish(msg);
}