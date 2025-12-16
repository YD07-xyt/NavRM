from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 基础配置
    pkg_your_pkg = get_package_share_directory('your_package_name')
    sensor_launch_path = os.path.join(pkg_your_pkg, 'launch', 'sensor_launch.py')
    rtabmap_params_path = os.path.join(pkg_your_pkg, 'config', 'rtabmap_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    # 3. RTAB-Map建图节点（加载通用+建图参数）
    rtabmap_mapping = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_params_path,          # 加载通用参数（rtabmap_common）
            {'~mode': 'mapping'},         # 覆盖为建图模式（优先级高于YAML）
            {'use_sim_time': use_sim_time}
        ],
        # 命名空间：匹配YAML中的rtabmap_mapping
        namespace='rtabmap_mapping',
        remappings=[
            ('scan', '/livox/lidar'),
            ('scan_cloud', '/livox/points'),
            ('rgb/image', '/rgb/image'),
            ('rgb/camera_info', '/rgb/camera_info')
        ]
    )

    # 4. RTAB-Map可视化
    rtabmap_viz = Node(
        package='rtabmap_ros',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/livox/lidar'),
            ('rgb/image', '/rgb/image')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use sim time'),
        rtabmap_mapping,
        rtabmap_viz
    ])