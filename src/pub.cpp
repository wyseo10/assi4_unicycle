// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
#include "cmd_publisher.h"

#pragma GCC push_options
#pragma GCC optimize("O0")

using namespace std::chrono_literals;
using std::placeholders::_1;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdPublisher>());


  rclcpp::shutdown();
  return 0;
}

#pragma GCC pop_options


    //Try 1. real_time 제어 
    //cnt++;
    //real_time = cnt * dt;

    //RCLCPP_INFO(this->get_logger(), "err_x, err_x: '[%lf] [%lf]'", err_x, err_x);
    //RCLCPP_INFO(this->get_logger(), "dt: '%lf'", dt);
    // RCLCPP_INFO(this->get_logger(), "real_time: '%lf'", real_time);

    // Trajectory generator
    // if(cnt == 400) cnt = 0;

    // if(real_time < 1)
    // {
    //   cmd_x += dt * square_dist;
    //   // RCLCPP_INFO(this->get_logger(), "right!");
    //   x_flag = true;
    //   y_flag = false;
      
    // }
    // else if (real_time < 2)
    // {
    //   cmd_y += dt * square_dist;
    //   // RCLCPP_INFO(this->get_logger(), "Up!");
    //   x_flag = false;
    //   y_flag = true;
    // }
    // else if (real_time < 3)
    // {
    //   cmd_x -= dt * square_dist;
    //   // RCLCPP_INFO(this->get_logger(), "Left!");
    //   x_flag = true;
    //   y_flag = false;
    // }
    // else
    // {
    //   cmd_y -= dt * square_dist;
    //   // RCLCPP_INFO(this->get_logger(), "Down!");
    //   x_flag = false;
    //   y_flag = true;    RCLCPP_INFO(this->get_logger(), "right!");

    //   // Error 
    //  if(x_flag)
    //   {
    //     err_x = cmd_x - real_x;
    //     err_y = 0;
    //   }
    //   else
    //   {
    //     err_x = 0;
    //     err_y = cmd_y - real_y;
    //   }
    //   // Ctrler
    //   cmd_vel.linear.x = err_x;
    //   cmd_vel.linear.y = 10;

    // Try 2. 
    // if(real_x < x_goal + threshold && real_x > x_goal - threshold &&
    //      real_y < y_goal + threshold && real_y > y_goal - threshold)
    // {
    //   cmd_vel.linear.x = x_velocity;
    //   cmd_vel.angular.z = 1000 * (atan2(y_goal, x_goal) - real_z);

    //   RCLCPP_INFO(this->get_logger(), "right!");
    // }

    // else if(sig == 1)
    // {
    //   cmd_vel.linear.x = 0;
    //   cmd_vel.linear.y = z_velocity;
    //   RCLCPP_INFO(this->get_logger(), "Up!");
    // }

    // else if(sig == 2)
    // {
    //   cmd_vel.linear.x = -x_velocity;
    //   cmd_vel.linear.y = 0;
    //   RCLCPP_INFO(this->get_logger(), "Left!");
    // }

    // else if(sig == 3)
    // {
    //   cmd_vel.linear.x = 0;
    //   cmd_vel.linear.y = -z_velocity;
    //   RCLCPP_INFO(this->get_logger(), "Down!");
      
    //   sig = 0;
    // }