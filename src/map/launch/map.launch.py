from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys

def generate_launch_description():
    pkg_name = "map"
    pkg_share = get_package_share_directory(pkg_name)
    param_config = os.path.join(pkg_share, "config", "grid_map.yaml")

    global_map = Node(
        package=pkg_name,
        executable='global_map_node',  # 和CMake中add_executable的名称一致
        name='global_map',
        namespace='map',
        output='screen',
        parameters=[param_config],
    )

    local_map = Node(
        package=pkg_name,
        executable='local_map_node',  # 和CMake中add_executable的名称一致
        name='local_map',
        namespace='map',
        output='screen',
        parameters=[param_config],
    )

    return LaunchDescription([global_map])