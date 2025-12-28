import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        parameters=[
            "src/location/odometer/point_lio/config/mid360.yaml"
        ],
    )
    ld = LaunchDescription()
    ld.add_action(start_point_lio_node)
    return ld