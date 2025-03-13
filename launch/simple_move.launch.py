import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='simple_move_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='rm_simpal_move',
                    plugin='rm_simpal_move::RMSimpleMove',
                    name='global_position_listener',
                    parameters=[
                        {'use_sim_time': False}
                    ]
                ),
            ],
            output='screen',
        ),
    ])