#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class Patrol : public rclcpp::Node {
public:
  Patrol()
      : Node("patrol_node"), direction_(0.0), is_turning_(false), yaw_(0.0),
        yaw_at_turn_start_(0.0) {

    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot_1/odom", 10,
        std::bind(&Patrol::odom_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    auto timer_period = std::chrono::milliseconds(100);
    timer_ = this->create_wall_timer(timer_period,
                                     std::bind(&Patrol::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Patrol Node Ready");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    if (is_turning_) {
      return;
    }

    // Only check straight ahead
    float front = msg->ranges[99];

    RCLCPP_INFO(this->get_logger(), "front=%.2f", front);

    if (front < 0.35) {

      // Find max range index in front 180° (indices 50-150)
      int max_idx = 99;
      float max_dist = 0.0;

      for (int j = 50; j <= 150; j++) {
        if (msg->ranges[j] > max_dist &&
            msg->ranges[j] != std::numeric_limits<float>::infinity()) {
          max_dist = msg->ranges[j];
          max_idx = j;
        }
      }

      // Convert to angle — naturally between -pi/2 and +pi/2
      direction_ = msg->angle_min + (max_idx * msg->angle_increment);

      // Snapshot yaw at turn start
      yaw_at_turn_start_ = yaw_;
      is_turning_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Front wall! max_idx=%d direction=%.2f yaw_start=%.2f",
                  max_idx, direction_, yaw_at_turn_start_);
    }
  }

  void timer_callback() {
    auto msg = geometry_msgs::msg::Twist();

    if (is_turning_) {
      double turned_so_far = yaw_ - yaw_at_turn_start_;

      if (std::abs(turned_so_far) < std::abs(direction_) - 0.1) {
        msg.linear.x = 0.1;
        msg.angular.z = direction_ / 2;
        RCLCPP_INFO(this->get_logger(), "Turning... turned=%.2f target=%.2f",
                    turned_so_far, direction_);
      } else {
        is_turning_ = false;
        RCLCPP_INFO(this->get_logger(), "Turn complete.");
      }
    } else {
      msg.linear.x = 0.1;
      msg.angular.z = 0.0;
    }

    publisher_->publish(msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double direction_;
  double yaw_;
  double yaw_at_turn_start_;
  bool is_turning_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}