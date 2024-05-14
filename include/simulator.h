#ifndef ROS2_TUTORIAL_SIMULATOR_H
#define ROS2_TUTORIAL_SIMULATOR_H

#include "rclcpp/rclcpp.hpp"
#include <turtlesim/msg/pose.hpp>

//TODO: Uncomment these!
//#include "visualization_msgs/msg/marker_array.hpp"
//#include "tf2_ros/transform_broadcaster.h"

class Simulator : public rclcpp::Node {
public:
    Simulator();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_poses;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> subs_pose;

    double robot_radius = 0.5;
    double offset = 1.0;
    size_t number_of_robots = 5;
    double ori = 5.544444561004639;
    std::vector<geometry_msgs::msg::Pose> poses;

    void timer_callback();

    void topic_callback(const turtlesim::msg::Pose &msg, size_t id);
};


#endif //ROS2_TUTORIAL_SIMULATOR_H
