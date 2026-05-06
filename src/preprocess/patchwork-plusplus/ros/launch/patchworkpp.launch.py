from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# This configuration parameters are not exposed thorugh the launch system, meaning you can't modify
# those throw the ros launch CLI. If you need to change these values, you could write your own
# launch file and modify the 'parameters=' block from the Node class.
class config:
    # TBU. Examples are as follows:
    max_range: float = 80.0
    # deskew: bool = False


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # tf tree configuration, these are the likely 3 parameters to change and nothing else
    base_frame = LaunchConfiguration("base_frame", default="base_link")

    # ROS configuration
    pointcloud_topic = LaunchConfiguration("cloud_topic")
    visualize = LaunchConfiguration("visualize", default="true")

    # Optional ros bag play
    bagfile = LaunchConfiguration("bagfile", default="")

    # Patchwork++ node
    patchworkpp_node = Node(
        package="patchworkpp",
        executable="patchworkpp_node",
        name="patchworkpp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
        parameters=[
            {
                # ========== ROS 节点配置 ==========
                "base_frame": base_frame,           # 机器人基座坐标系，用于转换点云
                "use_sim_time": use_sim_time,       # 是否使用仿真时间（Gazebo）
                
                # ========== 传感器安装参数 ==========
                "sensor_height": 0.53,              # 传感器安装高度（米）
                                                    # 根据实际 LiDAR 离地高度设置
                
                # ========== 地面估计核心参数 ==========
                "num_iter": 3,                      # PCA（主成分分析）迭代次数
                                                    # 越大越精确但计算量增加，范围：2-5
                "num_lpr": 20,                      # 最低点代表点数量
                                                    # 用于初始地面种子点选择，范围：10-50
                "num_min_pts": 0,                   # 每个 patch 最少点数
                                                    # 低于此值的 patch 不处理，0表示无限制
                "th_seeds": 0.3,                    # 地面种子点阈值（米）
                                                    # 低于传感器高度此值的点视为潜在地面点
                "th_dist": 0.125,                   # 地面厚度阈值（米）
                                                    # 点到此阈值内视为地面点
                
                # ========== 垂直结构参数（用于非地面点）==========
                "th_seeds_v": 0.25,                 # 垂直结构种子点阈值
                                                    # 用于初始垂直结构点选择
                "th_dist_v": 0.9,                   # 垂直结构厚度阈值（米）
                                                    # 垂直结构点的厚度判断
                
                # ========== 处理范围参数 ==========
                "max_range": 80.0,                  # 地面估计最大范围（米）
                                                    # 只处理此范围内的点，调小可提高性能
                "min_range": 1.0,                   # 地面估计最小范围（米）
                                                    # 忽略太近的点，避免车身干扰
                
                # ========== 地面似然估计参数 ==========
                "uprightness_thr": 0.101,           # 直立性阈值（0-1）
                                                    # 越高越严格，用于判断是否为地面点
                
                # ========== 调试参数 ==========
                "verbose": True,                    # 是否输出详细日志
                                                    # True 显示详细信息，False 静默运行
            }
        ],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("patchworkpp"), "rviz", "patchworkpp.rviz"]
            ),
        ],
        condition=IfCondition(visualize),
    )

    bagfile_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bagfile],
        output="screen",
        condition=IfCondition(PythonExpression(["'", bagfile, "' != ''"])),
    )
    
    return LaunchDescription(
        [
            patchworkpp_node,
            rviz_node,
            bagfile_play,
        ]
    )