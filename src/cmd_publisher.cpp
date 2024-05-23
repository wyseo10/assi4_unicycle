#include "cmd_publisher.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
  pub_cmd =
    this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd", 10);

  sub_pose = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "robot/pose", 10, std::bind(&CmdPublisher::topic_callback, this, _1));

  timer_ = this->create_wall_timer(
      10ms, std::bind(&CmdPublisher::timer_callback, this));
}

void CmdPublisher::topic_callback(const visualization_msgs::msg::MarkerArray &msg) {
    if(msg.markers.empty()) {
        return;
    }

    real_x = msg.markers[0].pose.position.x;
    real_y = msg.markers[0].pose.position.y;

    geometry_msgs::msg::Quaternion q = msg.markers[0].pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    real_theta = std::atan2(siny_cosp, cosy_cosp);
}

void CmdPublisher::timer_callback() {
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