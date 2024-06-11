#include "cmd_publisher.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
  // Publisher
  pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd", 10);

  // TF listener
  tf_buffer = todo;
  tf_listener = todo;

  // Timer
  timer_tf = this->create_wall_timer(
            10ms, std::bind(&CmdPublisher::timer_tf_callback, this));
  timer_cmd = this->create_wall_timer(
      10ms, std::bind(&CmdPublisher::timer_cmd_callback, this));
}

void CmdPublisher::timer_tf_callback() {
    // Simulator에서 broadcast한 tf를 통해 현재 로봇 상태를 받습니다

    geometry_msgs::msg::TransformStamped t;

    //TODO: implement this part!

    real_x = todo;
    real_y = todo;
    real_theta = todo;
}

void CmdPublisher::timer_cmd_callback() {
  // Goal assignment
  if (count == 0) {
    goal_x = 2;
    goal_y = 0;
    goal_theta = 0;
  } else if (count == 1) {
    goal_x = 2;
    goal_y = 0;
    goal_theta = 0.5 * pi;
  } else if (count == 2) {
    goal_x = 2;
    goal_y = 2;
    goal_theta = 0.5 * pi;
  } else if (count == 3) {
    goal_x = 2;
    goal_y = 2;
    goal_theta = pi;
  } else if (count == 4) {
    goal_x = 0;
    goal_y = 2;
    goal_theta = pi;
  } else if (count == 5) {
    goal_x = 0;
    goal_y = 2;
    goal_theta = 1.5 * pi;
  } else if (count == 6) {
    goal_x = 0;
    goal_y = 0;
    goal_theta = 1.5 * pi;
  } else if (count == 7) {
    goal_x = 0;
    goal_y = 0;
    goal_theta = 0;
  }

  // Error
  double err_x = goal_x - real_x;
  double err_y = goal_y - real_y;
  double err_dist = sqrt(err_x * err_x + err_y * err_y);

  double err_theta = goal_theta - real_theta;
  if (err_theta > 2 * pi) {
    err_theta -= 2 * pi;
  } else if (err_theta < -2 * pi) {
    err_theta += 2 * pi;
  }

  // State transition
  if ((count % 2 == 0 and err_dist < dist_threshold) or
      (count % 2 == 1 and err_theta < theta_threshold)) {
    count++;
    count = count % 8;
 
    // Reset error term to prevent error divergence
    sum_err_dist = 0;
    sum_err_theta = 0;
  }

  // PID control
  sum_err_dist += err_dist * dt;
  sum_err_theta += err_theta * dt;
  double delta_err_dist = (err_dist - prev_err_dist) / dt;
  double delta_err_theta = (err_theta - prev_err_theta) / dt;
  cmd_vel.linear.x = kP_dist * err_dist + kI_dist * sum_err_dist + kD_dist * delta_err_dist;
  cmd_vel.angular.z = kP_theta * err_theta + kI_theta * sum_err_theta + kD_theta * delta_err_theta;

  // Save the error for the next command
  prev_err_dist = err_dist;
  prev_err_theta = err_theta;

  // Publish!!!
  pub_cmd->publish(cmd_vel);
}