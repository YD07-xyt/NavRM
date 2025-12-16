import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

# cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
# cur_config_path = cur_path + '../config'
# livox_ros2_config_path = os.path.join(cur_config_path, 'MID360_config.json')

livox_ros2_config_path='src/io/livox_ros_driver2/config/MID360_config.json'
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": livox_ros2_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():
    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("small_point_lio"),
                    "config",
                    "mid360.yaml",
                ]
            )
        ],
    )
    #[0.127,0.075,-0.435,-0.102,0.5044,-1.57]
    static_base_link_to_livox_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-0.127",
            "--y",
            "-0.075",
            "--z",
            "0.435",
            "--roll",
            "0.102",
            "--pitch",
            "-0.5044",
            "--yaw",
            "1.57",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "livox_frame",
        ],
    )

    terrain_analysis_node = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        #arguments=["--ros-args", "--log-level", log_level],
        parameters=[  
            "src/location/preprocess/terrain_analysis/config/param.yaml"          
            # PathJoinSubstitution(
            #     [
            #         FindPackageShare("terrain_analysis"),
            #         "config",
            #         "param.yaml",
            #     ]
            # )
        ],
    )
    terrain_analysis_ext_node = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[  
            "src/location/preprocess/terrain_analysis_ext/config/param.yaml"         
            # PathJoinSubstitution(
            #     [
            #         FindPackageShare("terrain_analysis_ext"),
            #         "config",
            #         "param.yaml",
            #     ]
            # )
        ],
    )
    pointcloud_to_laserscan_node =Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in',  ['/livox/lidar/pointcloud']),
            # remappings=[('cloud_in',  ['cloud_registered_body']),
            remappings=[('cloud_in',  ['/terrain_map_ext']),            
                        ('scan',  ['/scan'])],
            parameters=[{
                'target_frame': 'odom',
                'transform_tolerance': 0.01, 
                'min_height': -1.5,
                'max_height': 6.0,
                'min_intensity': 0.1,
                'max_intensity': 2.0,
                'angle_min': -3.14159, # -M_PI
                'angle_max': 3.14159, # M_PI
                'angle_increment': 0.0043,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }])
    rviz2_node = Node(
        package="rviz2",          # rviz2所属功能包（系统自带）
        executable="rviz2",       # 可执行文件名（固定为rviz2）
        name="rviz2",             # 节点名（自定义，可省略）
        output="screen",          # 日志输出到终端（调试用）
        arguments=[
            "-d",PathJoinSubstitution(
                [
                    FindPackageShare("bringup"),
                    "rviz",
                    "map.rviz",
                ]
            ),
            "-f", "map"]  # 启动参数：-f 指定固定帧（适配你的点云frame_id）
    )
    return LaunchDescription([livox_driver_node,small_point_lio_node, 
                              static_base_link_to_livox_frame,terrain_analysis_node,
                              terrain_analysis_ext_node,pointcloud_to_laserscan_node,rviz2_node])