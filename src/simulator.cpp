#include "simulator.h"

using namespace std::chrono_literals;

Simulator::Simulator() : Node("simulator") {
    // ROS publisher
    pub_poses = this->create_publisher<visualization_msgs::msg::MarkerArray>("/poses", 10);

    // ROS subscriber
    subs_pose.resize(number_of_robots);
    for (size_t id = 0; id < number_of_robots; id++) {
        std::function<void(const turtlesim::msg::Pose msg)> fcn =
                std::bind(&Simulator::topic_callback, this, std::placeholders::_1, id);
        subs_pose[id] = this->create_subscription<turtlesim::msg::Pose>(
                "/turtlesim" + std::to_string(id) + "/turtle1/pose", 10, fcn);
    }

    // ROS tf publisher
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ROS timer
    timer_ = this->create_wall_timer(
            10ms, std::bind(&Simulator::timer_callback, this));

    // Poses
    poses.resize(number_of_robots);
}

void Simulator::timer_callback() {
    visualization_msgs::msg::MarkerArray msg;

    for(size_t id = 0; id < number_of_robots; id++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = "pose";
        marker.id = (int) id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = poses[id].position;
        marker.scale.x = robot_radius;
        marker.scale.y = robot_radius;
        marker.scale.z = robot_radius;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        msg.markers.emplace_back(marker);
    }

    pub_poses->publish(msg);
}

void Simulator::topic_callback(const turtlesim::msg::Pose &msg, size_t id) {
    poses[id].position.x = msg.x - ori + (double)id * offset;
    poses[id].position.y = msg.y - ori;
    poses[id].position.z = 0;
}