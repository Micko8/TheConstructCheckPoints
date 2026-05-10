#pragma once

#include "robot_gui/magic_subscriber_class.h"
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>


class RobotGUI {
public:
  RobotGUI();
  void run();

private:
  const std::string window_name_ = "Robot GUI";
  ros::NodeHandle nh_;
  /*Info/CmdVel/Odom Subscribing*/
  MagicSubscriber<robotinfo_msgs::RobotInfo10Fields> robot_info_sub_;
  std::vector<std::string>
  robotInfoToLines(const robotinfo_msgs::RobotInfo10Fields &msg) const;

  MagicSubscriber<geometry_msgs::Twist> cmd_vel_sub_;

  MagicSubscriber<nav_msgs::Odometry> odom_sub_;

  /*CmdVel Publishing*/
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist command_twist_;
  const double linear_step_ = 0.1;
  const double angular_step_ = 0.1;
  const double max_linear_velocity_ = 1.0;
  const double max_angular_velocity_ = 1.0;
  double clamp(double value, double min_value, double max_value) const;
  void publishCommand();

  /*Service Get Distance*/
  ros::ServiceClient get_distance_client_;
  ros::ServiceClient reset_distance_client_;
  std::string distance_response_ = "Press Call";

};
