from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        
        # 点云转激光扫描节点（优化降噪版）
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/terrain_map_ext'),   # 输入：非地面点云
                ('scan', '/obstacle_scan')                 # 输出：激光扫描数据
            ],
            parameters=[{
                # ========== 坐标变换参数 ==========
                'target_frame': 'world',                   # 目标坐标系
                'transform_tolerance': 0.01,               # 变换容差（秒）
                
                # ========== 高度过滤（去除垂直方向噪声）==========
                'min_height': 0.1,                         # 最小高度0.1米（去除地面附近噪声）
                'max_height': 2.5,                         # 最大高度2.5米（只关注常见障碍物高度）
                
                # ========== 强度过滤（可选，如果点云有强度信息）==========
                'min_intensity': 0.0,                      # 最小强度（不过滤）
                'max_intensity': 2.0,                      # 最大强度
                
                # ========== 角度范围（360度全向扫描）==========
                'angle_min': -3.14159,                     # -180度
                'angle_max': 3.14159,                      # +180度
                'angle_increment': 0.0043,                 # 角度分辨率约0.25度
                
                # ========== 时间参数 ==========
                'scan_time': 0.1,                          # 扫描周期0.1秒（10Hz）
                
                # ========== 距离过滤（关键降噪参数）==========
                'range_min': 0.5,                          # 最小距离0.5米（去除车身自身点云）
                'range_max': 12.0,                         # 最大距离12米（减少远处噪声）
                
                # ========== 其他参数 ==========
                'use_inf': True,                           # 使用无限远点
                'inf_epsilon': 1.0,                        # 无限远点替换值
                
                # ========== 降采样参数（减少数据量）==========
                'point_cloud_step': 2,                     # 每隔一个点取一个（降采样）
            }],
            name='pointcloud_to_laserscan'
        )
    ])