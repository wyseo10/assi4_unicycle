#include "simulator.h"

using namespace std::chrono_literals;

Simulator::Simulator() : Node("simulator") {
    // ROS publisher
    pub_poses = this->create_publisher<visualization_msgs::msg::MarkerArray>("/poses", 10);

    // ROS subscriber
    subs_pose.resize(number_of_robots);
    for (size_t id = 0; id < number_of_robots; id++) {
        // This allows the callback function to get argument
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

    //TODO: implement this part!

    pub_poses->publish(msg);
}

void Simulator::topic_callback(const turtlesim::msg::Pose &msg, size_t id) {
    poses[id].position.x = msg.x - ori + (double)id * offset;
    poses[id].position.y = msg.y - ori;
    poses[id].position.z = 0;

    //TODO: implement this!
    //Hint: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // Yaw angle of the robot = msg.theta
    // poses[id].orientation = ????
}