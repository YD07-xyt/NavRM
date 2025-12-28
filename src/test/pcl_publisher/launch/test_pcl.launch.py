from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="pcl_publisher",
        executable="pcl_publisher_node",
        namespace="test",
        output="screen",
        parameters=[],
    )

    return LaunchDescription([node])