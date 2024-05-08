#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <turtlesim/msg/pose.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>

class CmdPublisher : public rclcpp::Node
{
public:
  CmdPublisher();
  
private:
 void topic_callback(const turtlesim::msg::Pose &msg);

 void timer_callback();

  //position
  double ori = 5.544444561004639;
  double real_x = 0;
  double real_y = 0;
  double real_z = 0;
  double threshold = 0.1;
  
  double x_goal = 2;
  double y_goal = 0;
  double z_goal = 0;

  //double cmd_x = 0;
  //double cmd_y = 0;

  //error
  double err_x = 0;
  double err_y = 0;
  double err_theta = 0;

  //trajectory
  double square_dist = 3;
  double dt = 0.01;
  double cnt = 0;
  double real_time=0;

  double kP_x = 1;
  double kP_y = 2;

  bool x_flag = false;
  bool y_flag = false;

  double x_velocity = 3;
  double z_velocity = 3;
  int sig = 0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;

  //-Init variables
  geometry_msgs::msg::Twist cmd_vel;
};
