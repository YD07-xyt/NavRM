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

    static_base_link_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-0",
            "--y",
            "-0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "-0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
    )
    return LaunchDescription([node])