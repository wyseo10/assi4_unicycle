#include "cmd_publisher.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&CmdPublisher::topic_callback, this, _1));
    
    timer_ = this->create_wall_timer(
    10ms, std::bind(&CmdPublisher::timer_callback, this));
  }


  void CmdPublisher::topic_callback(const turtlesim::msg::Pose &msg)
  {
    real_x = msg.x - ori;
    real_y = msg.y - ori;
    real_z = msg.theta;
    //RCLCPP_INFO(this->get_logger(), "x position: '%lf'", real_x);
    //RCLCPP_INFO(this->get_logger(), "y position: '%lf'", real_y);
  }


   void CmdPublisher::timer_callback()
  {
    err_x = x_goal - real_x;
    err_y = y_goal - real_y;

    double linear_err = sqrt(err_x * err_x + err_y * err_y);

    z_goal = atan2(err_y, err_x);
    err_theta = z_goal - real_z;

  if(err_theta > 3.141592 * 2)
  {
    err_theta -= 3.141592 * 2;
  }
  else if (err_theta < - 3.141592*2)
  {
    err_theta += 3.141592 * 2;

  }

    // if(err_theta < 0.05)
    // {
    // cmd_vel.linear.x = linear_err;
    // cmd_vel.angular.z = 0;
    // }
    // else{
    // cmd_vel.linear.x = 0;
    // cmd_vel.angular.z = err_theta;
    // }

    cmd_vel.linear.x = linear_err;
    cmd_vel.angular.z = err_theta;
    
    if(linear_err < 0.1) cnt++;

    if(cnt == 1)
    {
      x_goal =2;
      y_goal =2;
      RCLCPP_INFO(this->get_logger(), "1");

    }
    
    if(cnt == 2)
    {
      x_goal =-2;
      y_goal =2;
      RCLCPP_INFO(this->get_logger(), "2");

    }

    if(cnt == 3)
    {
      x_goal =-2;
      y_goal =-2;
      RCLCPP_INFO(this->get_logger(), "3");

    }

    if(cnt == 4)
    {
      x_goal =2;
      y_goal =-2;
      cnt = 0;
      RCLCPP_INFO(this->get_logger(), "4");

    }
    //RCLCPP_INFO(this->get_logger(), "cmd_vel.linear.x: '%lf'", cmd_vel.linear.x);
    RCLCPP_INFO(this->get_logger(), "err_theta: '%lf'", err_theta);

    //publish!!!
    publisher_->publish(cmd_vel);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  }