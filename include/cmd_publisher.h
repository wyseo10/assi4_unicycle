#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/msg/pose.hpp>

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class CmdPublisher : public rclcpp::Node {
public:
  CmdPublisher();

private:
  void topic_callback(const turtlesim::msg::Pose &msg);

  void timer_callback();

  // Parameters
  double dt = 0.01;
  double dist_threshold = 0.05;
  double theta_threshold = 0.01;
  const double pi = 3.141592;

  // Robot state
  double ori = 5.544444561004639;
  double real_x = 0;
  double real_y = 0;
  double real_theta = 0;

  // Goal state
  double goal_x = 2;
  double goal_y = 0;
  double goal_theta = 0;

  // Mission state and parameter
  int count = 0;

  // PID error
  double prev_err_dist = 0;
  double prev_err_theta = 0;
  double sum_err_dist = 0;
  double sum_err_theta = 0;

  // PID gain
  double kP_dist = 1;
  double kI_dist = 0.05;
  double kD_dist = 0.01;
  double kP_theta = 1;
  double kI_theta = 0.05;
  double kD_theta = 0.01;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;

  // Init variables
  geometry_msgs::msg::Twist cmd_vel;
};
