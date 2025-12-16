import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    aim_driver_node = Node(
        package='aim_driver',
        executable='aim_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([aim_driver_node])
