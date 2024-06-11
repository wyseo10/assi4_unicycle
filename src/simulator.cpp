#include "simulator.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

Simulator::Simulator() : Node("simulator") {
    // ROS publisher
    pub_pose = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot/pose", 10);

    // ROS subscriber
    sub_cmd = this->create_subscription<geometry_msgs::msg::Twist>(
            "robot/cmd", 10, std::bind(&Simulator::cmd_callback, this, _1));

    // ROS tf publisher
    tf_broadcaster = todo;

    // ROS timer
    timer_ = this->create_wall_timer(
            10ms, std::bind(&Simulator::timer_callback, this));
}

void Simulator::timer_callback() {
    // 10ms마다 아래 함수들을 호출합니다.
    update_state();
    publish_marker_pose();
    broadcast_tf();
}

void Simulator::cmd_callback(const geometry_msgs::msg::Twist &msg) {
    // CmdPublisher 노드로부터 control input을 받아옵니다.
    cmd_vel.v = todo;
    cmd_vel.w = todo;
}

void Simulator::update_state() {
    //Hint: 이 함수는 10ms 시간 주기로 호출됩니다.
    //로봇의 state (x,y,theta)를 cmd_vel을 사용하여 업데이트 해야합니다.

    double x_dot = todo;
    double y_dot = todo;
    double theta_dot = todo;

    state.x = state.x + x_dot * todo;
    state.y = state.y + y_dot * todo;
    state.theta = state.theta + todo;
}

void Simulator::publish_marker_pose() {
    // 로봇의 현재 상태를 rviz에 시각화합니다.
    visualization_msgs::msg::MarkerArray msg;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = todo;
    marker.ns = "pose";
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = todo;
    marker.action = todo;
    marker.pose.position.x = todo;
    marker.pose.position.y = todo;
    marker.pose.position.z = todo;
    marker.pose.orientation.w = todo;
    marker.pose.orientation.x = todo;
    marker.pose.orientation.y = todo;
    marker.pose.orientation.z = todo;
    marker.scale.x = robot_scale;
    marker.scale.y = robot_scale;
    marker.scale.z = robot_scale;
    marker.color.a = 1;
    msg.markers.emplace_back(marker);

    pub_pose->publish(msg);
}

void Simulator::broadcast_tf() {
    // 로봇의 현재 상태를 CmdPublisher node에 전달하기 위해 tf를 사용합니다.
    geometry_msgs::msg::TransformStamped t;

    //TODO: implement this part!

    tf_broadcaster->sendTransform(t);
}
