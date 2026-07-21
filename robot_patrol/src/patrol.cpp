#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    RCLCPP_INFO(this->get_logger(), "Patrol Node Init");

    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", 10,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
#if 0
    std::map<std::string, std::pair<int, int>> sectors = {
        {"Front", {34, 166}},    {"Right", {34, 66}},
        {"Front_Right", {67, 100}}, {"Front_Left", {101, 133}},
        {"Left", {134, 166}},       {"Left_Rear", {167, 199}}};

    std::map<std::string, float> min_distances;
    for (const auto &sector : sectors) {
      min_distances[sector.first] = std::numeric_limits<float>::infinity();
    }

    for (const auto &sector : sectors) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;
      if (start_idx < static_cast<int>(msg->ranges.size()) &&
          end_idx < static_cast<int>(msg->ranges.size())) {
        float min_val = std::numeric_limits<float>::infinity();
        for (int i = start_idx; i <= end_idx; ++i) {
          if (msg->ranges[i] < min_val) {
            min_val = msg->ranges[i];
          }
        }
        min_distances[sector.first] = min_val;
      }
    }

    float obstacle_threshold = 0.8f;
    std::map<std::string, bool> detections;
    for (const auto &min_dist : min_distances) {
      detections[min_dist.first] = min_dist.second < obstacle_threshold;
    }

    // If no active mission, just do obstacle avoidance exploration
    if (!mission_active_) {
      auto action = geometry_msgs::msg::Twist();
      if (detections["Front_Left"] || detections["Front_Right"]) {
        if (!turning_) {
          turning_ = true;
          turn_direction_ = -0.5;
        }
        action.angular.z = turn_direction_;
        RCLCPP_INFO(this->get_logger(),
                    "Obstacle ahead, turning to clear path.");
      } else {
        turning_ = false;
        if (detections["Left"]) {
          action.linear.x = 0.2;
          action.angular.z = -0.3;
        } else if (detections["Right"]) {
          action.linear.x = 0.2;
          action.angular.z = 0.3;
        } else if (detections["Right_Rear"] || detections["Left_Rear"]) {
          action.linear.x = 0.3;
        } else {
          // action.linear.x = 0.5;
        }
      }
      publisher_->publish(action);
      return;
    }

    // Active mission: navigate to target with obstacle avoidance
    if (detections["Front_Left"] || detections["Front_Right"]) {
      // Obstacle in front: turn to avoid
      auto action = geometry_msgs::msg::Twist();
      if (!turning_) {
        turning_ = true;
        turn_direction_ = -0.5;
      }
      action.angular.z = turn_direction_;
      publisher_->publish(action);
      RCLCPP_INFO(this->get_logger(), "Obstacle ahead, turning to clear path.");
    } else {
      // No front obstacle: navigate towards goal
      turning_ = false;
      navigate_to_goal();
    }
#endif
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
