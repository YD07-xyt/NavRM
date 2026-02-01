from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = "planning_env"
    pkg_share = get_package_share_directory(pkg_name)
    param_config = os.path.join(pkg_share, "config", "env.yaml")

    planner_map = Node(
        package=pkg_name,
        executable='planning_env_node',
        name='planner_map',
        namespace='planner',
        output='screen',
        parameters=[param_config],
    )

    return LaunchDescription([planner_map])