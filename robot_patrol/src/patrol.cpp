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
        target_yaw_(0.0) {

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

    // Straight ahead only: index 99 = exactly 0 rad
    float front = msg->ranges[99];

    // Very tight side zones — only for collision avoidance
    float min_right = std::numeric_limits<float>::infinity();
    float min_left = std::numeric_limits<float>::infinity();

    for (int i = 90; i <= 98; i++) { // tight right of center
      if (msg->ranges[i] < min_right &&
          msg->ranges[i] != std::numeric_limits<float>::infinity())
        min_right = msg->ranges[i];
    }

    for (int i = 100; i <= 109; i++) { // tight left of center
      if (msg->ranges[i] < min_left &&
          msg->ranges[i] != std::numeric_limits<float>::infinity())
        min_left = msg->ranges[i];
    }

    RCLCPP_INFO(this->get_logger(), "front=%.2f right=%.2f left=%.2f", front,
                min_right, min_left);

    if (front < 0.35 || min_right < 0.22 || min_left < 0.22) {

      if (front < 0.35) {
        // Front wall: find safest direction from max range
        int max_idx = 99;
        float max_dist = 0.0;
        for (int j = 50; j <= 150; j++) {
          if (msg->ranges[j] > max_dist &&
              msg->ranges[j] != std::numeric_limits<float>::infinity()) {
            max_dist = msg->ranges[j];
            max_idx = j;
          }
        }
        bool is_corner = std::abs(min_right - min_left) < 0.05;
        direction_ =
            is_corner ? 1.0 : msg->angle_min + (max_idx * msg->angle_increment);
        if (std::abs(direction_) < 0.5)
          direction_ = direction_ >= 0 ? 0.5 : -0.5;

      } else if (min_right < 0.22) {
        direction_ = 0.5; // too close on right → nudge left

      } else if (min_left < 0.22) {
        direction_ = -0.5; // too close on left → nudge right
      }

      target_yaw_ = yaw_ + direction_;
      target_yaw_ = std::fmod(target_yaw_ + M_PI, 2 * M_PI) - M_PI;
      is_turning_ = true;

      RCLCPP_INFO(this->get_logger(), "Turning! direction=%.2f target_yaw=%.2f",
                  direction_, target_yaw_);
    }
  }

  void timer_callback() {
    auto msg = geometry_msgs::msg::Twist();

    if (is_turning_) {
      double yaw_error = target_yaw_ - yaw_;
      yaw_error = std::fmod(yaw_error + M_PI, 2 * M_PI) - M_PI;

      if (std::abs(yaw_error) > 0.1) {
        msg.linear.x = 0.1;
        msg.angular.z = direction_ / 2;
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
  double target_yaw_;
  bool is_turning_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}