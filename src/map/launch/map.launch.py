from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os 
def generate_launch_description():
    pkg =get_package_share_directory('map')
    #param_config =os.path.join(pkg,'config','param.yaml')
    global_map = Node(
        package=pkg,
        executable='global_map',
        name='global_map',
        namespace='map',
        output='screen',
        parameters=[],
    )
    local_map = Node(
        package=pkg,
        executable='local_map',
        name='local_map',
        namespace='map',
        output='screen',
        parameters=[],
    )
    return LaunchDescription([global_map,local_map])