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
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

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
                # ROS node configuration
                "base_frame": base_frame,
                "use_sim_time": use_sim_time,
                # Patchwork++ configuration
                 #传感器安装高度（米）
                "sensor_height": 0.363,
                
                #PCA 地面平面估计迭代次数 迭代次数越多，地面拟合越精准，但计算耗时增加
                "num_iter": 2,  # Number of iterations for ground plane estimation using PCA.
                
                #最低点代表点的最大数量	选取每个 patch 中最低的 N 个点作为地面种子点
                "num_lpr": 15,  # Maximum number of points to be selected as lowest points representative.
                
                #每个 patch 中地面点的最小数量	低于该值的 patch 不参与地面拟合
                "num_min_pts": 8,  # Minimum number of points to be estimated as ground plane in each patch.
                
                #初始地面种子点的高度阈值（米）	筛选初始地面点的高度范围
                "th_seeds": 0.05,
                # threshold for lowest point representatives using in initial seeds selection of ground points.
                
                #地面厚度阈值（米）	判断点是否为地面点的核心阈值（点到拟合平面的距离）
                "th_dist": 0.05,  # threshold for thickness of ground.
                
                #垂直结构初始种子点高度阈值	筛选垂直结构（如柱子、墙体）的初始种子点
                "th_seeds_v": 0.25,
                # threshold for lowest point representatives using in initial seeds selection of vertical structural points.
                
                #垂直结构厚度阈值	判断点是否为垂直结构的距离阈值
                "th_dist_v": 0.9,  # threshold for thickness of vertical structure.
                
                #地面估计的最大范围（米）	只处理该范围内的点云，减少计算量
                "max_range": 6.0,  # max_range of ground estimation area
                
                #地面估计的最小范围（米）	过滤近距离噪声点（如传感器自身、车辆底盘）
                "min_range": 0.2,  # min_range of ground estimation area
                
                #垂直度阈值（GLE）	用于判断点是否属于垂直结构的垂直度指标
                "uprightness_thr": 0.101,
                # threshold of uprightness using in Ground Likelihood Estimation(GLE). Please refer paper for more information about GLE.
                
                #是否输出详细日志
                "verbose": True,  # display verbose info
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
