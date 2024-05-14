from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    number_of_sim = 5

    ld = LaunchDescription()
    for sim_id in range(0, number_of_sim):
        simulator_node = Node(
            namespace = 'turtlesim' + str(sim_id),
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'
        )
        ld.add_action(simulator_node)

        controller_node = Node(
            namespace= 'turtlesim' + str(sim_id),
            package= 'ros2_tutorial',
            executable= 'cmd_publisher_node',
            output='screen'
        )
        ld.add_action(controller_node)
    return ld
