import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'
livox_config_yaml = os.path.join(get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": livox_config_yaml},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    pkg_nav2 = get_package_share_directory('nav2')

    # Super LIO 配置
    pkg_super_lio = get_package_share_directory('super_lio')
    lio_config_yaml = os.path.join(pkg_super_lio, 'config', 'livox_360.yaml')
   
    # 1. Livox 驱动节点（最先启动）
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    
    # 2. Super LIO 节点
    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        name='super_lio_node',
        output='screen',
        parameters=[lio_config_yaml],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # 3. 点云转激光扫描节点
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/terrain_map_ext'),            
                    ('scan', '/obstacle_scan')],
        parameters=[{
            'target_frame': 'world',
            'transform_tolerance': 0.01, 
            'min_height': -1.5,
            'max_height': 6.0,
            'min_intensity': 0.1,
            'max_intensity': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )
    
    # 4. 假的速度变换节点
    fake_vel_transform_node = Node(
        package="fake_vel_transform",
        executable="fake_vel_transform_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # 5. Patchwork++ launch 文件（在 Super LIO 之后，SLAM 之前）
    pkg_patchworkpp = get_package_share_directory('patchworkpp')
    patchworkpp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_patchworkpp, 'launch', 'patchworkpp.launch.py')
        ),
        launch_arguments={
            'visualize': 'false',
            'use_sim_time': use_sim_time,
            'cloud_topic': '/lio/cloud_world',
            'base_frame': 'world'
        }.items()
    )
    
    # 6. SLAM Toolbox launch 文件（在 Patchwork++ 之后）
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'slam_toolbox.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_nav2, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )
    
    # 7. Nav2 launch 文件（最后启动）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_nav2, 'config', 'nav2_param.yaml'),
            'autostart': 'true'
        }.items()
    )
    
    # 使用 TimerAction 实现顺序启动
    # 启动顺序和时间表：
    # 0秒: Livox 驱动
    # 2秒: Fake Vel Transform
    # 5秒: Super LIO（等待点云数据）
    # 8秒: PointCloud to LaserScan
    # 11秒: Patchwork++（处理地面分割，在 SLAM 之前）
    # 15秒: SLAM Toolbox（使用 Patchwork++ 处理后的数据）
    # 22秒: Nav2（等待地图建立）
    
    delayed_fake_vel = TimerAction(
        period=0.0,
        actions=[fake_vel_transform_node]
    )
    
    delayed_super_lio = TimerAction(
        period=1.0,
        actions=[super_lio_node]
    )
    
    delayed_pointcloud = TimerAction(
        period=1.9,
        actions=[pointcloud_to_laserscan_node]
    )
    
    delayed_patchworkpp = TimerAction(
        period=1.4,  # Patchwork++ 在 SLAM 之前启动
        actions=[patchworkpp_launch]
    )
    
    delayed_slam = TimerAction(
        period=2.1,  # SLAM 在 Patchwork++ 之后启动
        actions=[slam_toolbox_launch]
    )
    
    delayed_nav2 = TimerAction(
        period=2.5,  # Nav2 最后启动
        actions=[nav2_launch]
    )
    
    return LaunchDescription([
        livox_driver_node,
        delayed_fake_vel,
        delayed_super_lio,
        delayed_pointcloud,
        delayed_patchworkpp,    # Patchwork++ 先启动
        delayed_slam,           # 然后 SLAM
        #delayed_nav2,           # 最后 Nav2
    ])