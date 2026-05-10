#include "robot_gui/robot_gui.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui");
  RobotGUI gui;
  gui.run();
  return 0;
}