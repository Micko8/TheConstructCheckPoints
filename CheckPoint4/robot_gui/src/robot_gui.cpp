#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include "robot_gui/robot_gui.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>


RobotGUI::RobotGUI()
    : nh_(), robot_info_sub_(nh_, "/robot_info"), cmd_vel_sub_(nh_, "/cmd_vel"),
      odom_sub_(nh_, "/odom") {
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  get_distance_client_ = nh_.serviceClient<std_srvs::Trigger>("/get_distance");
  reset_distance_client_ = nh_.serviceClient<std_srvs::Empty>("/reset_distance");
}

void RobotGUI::run() {
  cv::Mat frame = cv::Mat(720, 420, CV_8UC3);

  cvui::init(window_name_);

  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);

    cvui::window(frame, 20, 20, 380, 120, "General Info");
    robotinfo_msgs::RobotInfo10Fields robot_info_msg;
    /*General Infos Aera*/
    if (robot_info_sub_.getLatest(robot_info_msg)) {
      const std::vector<std::string> lines = robotInfoToLines(robot_info_msg);

      int y = 55;
      for (const std::string &line : lines) {
        cvui::text(frame, 35, y, line);
        y += 18;
      }
    }

    /*Buttons Cmd Vel*/
    if (cvui::button(frame, 160, 170, 100, 40, "Forward")) {
      command_twist_.linear.x += linear_step_;
      publishCommand();
    }

    if (cvui::button(frame, 160, 260, 100, 40, "Backward")) {
      command_twist_.linear.x -= linear_step_;
      publishCommand();
    }

    if (cvui::button(frame, 50, 215, 100, 40, "Left")) {
      command_twist_.angular.z += angular_step_;
      publishCommand();
    }

    if (cvui::button(frame, 270, 215, 100, 40, "Right")) {
      command_twist_.angular.z -= angular_step_;
      publishCommand();
    }

    if (cvui::button(frame, 160, 215, 100, 40, "Stop")) {
      command_twist_.linear.x = 0.0;
      command_twist_.angular.z = 0.0;
      publishCommand();
    }

    cvui::window(frame, 20, 330, 180, 60, "Linear velocity:");
    cvui::window(frame, 220, 330, 180, 60, "Angular velocity:");

    geometry_msgs::Twist cmd_vel_msg;
    if (cmd_vel_sub_.getLatest(cmd_vel_msg)) {
      cvui::printf(frame, 55, 365, 0.4, 0xff0000, "%.2f m/sec",
                   cmd_vel_msg.linear.x);
      cvui::printf(frame, 255, 365, 0.4, 0xff0000, "%.2f rad/sec",
                   cmd_vel_msg.angular.z);
    }

    cvui::text(frame, 55, 410, "Estimated Robot Position Based on Odometry");

    cvui::window(frame, 20, 430, 120, 90, "X");
    cvui::window(frame, 150, 430, 120, 90, "Y");
    cvui::window(frame, 280, 430, 120, 90, "Z");

    nav_msgs::Odometry odom_msg;
    if (odom_sub_.getLatest(odom_msg)) {
      cvui::printf(frame, 55, 475, 0.9, 0xcccccc, "%d",
                   static_cast<int>(odom_msg.pose.pose.position.x));
      cvui::printf(frame, 185, 475, 0.9, 0xcccccc, "%d",
                   static_cast<int>(odom_msg.pose.pose.position.y));
      cvui::printf(frame, 315, 475, 0.9, 0xcccccc, "%d",
                   static_cast<int>(odom_msg.pose.pose.position.z));
    }

    cvui::text(frame, 20, 540, "Distance Travelled");

    if (cvui::button(frame, 20, 560, 90, 45, "Call")) {
      std_srvs::Trigger srv;
      if (get_distance_client_.call(srv)) {
        distance_response_ = srv.response.message;
      }
    }

    if (cvui::button(frame, 20, 610, 90, 45, "Reset")) {
      std_srvs::Empty srv;
      if (reset_distance_client_.call(srv)) {
        distance_response_ = "0.00";
      }
    }

    cvui::window(frame, 120, 560, 280, 95, "Distance in meters:");
    cvui::text(frame, 330, 615, distance_response_);

    cvui::imshow(window_name_, frame);
    if (cv::waitKey(20) == 27) {
      break;
    }

    ros::spinOnce();
  }
}

std::vector<std::string>
RobotGUI::robotInfoToLines(const robotinfo_msgs::RobotInfo10Fields &msg) const {
  std::vector<std::string> lines;

  if (!msg.data_field_01.empty())
    lines.push_back(msg.data_field_01);
  if (!msg.data_field_02.empty())
    lines.push_back(msg.data_field_02);
  if (!msg.data_field_03.empty())
    lines.push_back(msg.data_field_03);
  if (!msg.data_field_04.empty())
    lines.push_back(msg.data_field_04);
  if (!msg.data_field_05.empty())
    lines.push_back(msg.data_field_05);
  if (!msg.data_field_06.empty())
    lines.push_back(msg.data_field_06);
  if (!msg.data_field_07.empty())
    lines.push_back(msg.data_field_07);
  if (!msg.data_field_08.empty())
    lines.push_back(msg.data_field_08);
  if (!msg.data_field_09.empty())
    lines.push_back(msg.data_field_09);
  if (!msg.data_field_10.empty())
    lines.push_back(msg.data_field_10);

  return lines;
}

double RobotGUI::clamp(double value, double min_value, double max_value) const {
  if (value < min_value) {
    return min_value;
  }

  if (value > max_value) {
    return max_value;
  }

  return value;
}

void RobotGUI::publishCommand() {
  command_twist_.linear.x = clamp(command_twist_.linear.x,
                                  -max_linear_velocity_, max_linear_velocity_);

  command_twist_.angular.z = clamp(
      command_twist_.angular.z, -max_angular_velocity_, max_angular_velocity_);

  cmd_vel_pub_.publish(command_twist_);
}
