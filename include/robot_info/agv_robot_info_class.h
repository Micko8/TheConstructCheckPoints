#ifndef AGV_ROBOT_INFO_CLASS_H
#define AGV_ROBOT_INFO_CLASS_H
#include "robot_info/agv_robot_info_class.h"
#include "robot_info/hydraulic_system_monitor.h"
#include "robot_info/robot_info_class.h"

class AGVRobotInfo : public RobotInfo {
private:
  std::string maximum_payload;
  HydraulicSystemMonitor hydroMonitorObject;

public:
  AGVRobotInfo(void);
  void publish_data() override;
};

#endif