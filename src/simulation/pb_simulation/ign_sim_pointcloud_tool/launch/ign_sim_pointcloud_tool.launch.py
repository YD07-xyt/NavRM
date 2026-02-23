import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description(): 
    pkg_name = "ign_sim_pointcloud_tool"
    pkg_share = get_package_share_directory(pkg_name)
    param_file = os.path.join(pkg_share, "config", "param.yaml")
    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        parameters=[param_file],
    )


    ld = LaunchDescription()
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_velodyne_convert_tool)

    return ld