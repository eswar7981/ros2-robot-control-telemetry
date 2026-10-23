
"""Launch the robot simulator node."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the robot simulator."""
    parameters_file_path = os.path.join(
        get_package_share_directory('robot_control'),
        'config',
        'params.yaml'
    )

    simulator_node = Node(
        package='robot_control',
        executable='robot_simulator_node',
        name='robot_simulator_node',
        output='screen',
        parameters=[parameters_file_path]
    )

    return LaunchDescription([simulator_node])
