from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # -------------------------- 声明启动参数 --------------------------
    # 声明可外部传入的参数，方便灵活配置，不传入则使用默认值
    declare_map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='TF 父坐标系（地图坐标系）'
    )

    declare_odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='TF 子坐标系（里程计坐标系）'
    )

    declare_pcd_path_arg = DeclareLaunchArgument(
        'pcd_path',
        # 默认路径：功能包下的 pcd 文件夹中的 rmuc_2026.pcd
        default_value=PathJoinSubstitution([
            FindPackageShare('bringup'),  # 替换为你的功能包名称
            'pcd',
            'rmuc_2026.pcd'
        ]),
        description='地图 PCD 文件的路径'
    )

    declare_num_threads_arg = DeclareLaunchArgument(
        'num_threads',
        default_value='4',
        description='GICP 配准使用的线程数'
    )

    declare_num_neighbors_arg = DeclareLaunchArgument(
        'num_neighbors',
        default_value='20',
        description='计算点云协方差的邻域点数'
    )

    declare_leaf_size_arg = DeclareLaunchArgument(
        'leaf_size',
        default_value='0.25',
        description='点云体素降采样的分辨率（米）'
    )

    declare_max_dist_sq_arg = DeclareLaunchArgument(
        'max_dist_sq',
        default_value='1.0',
        description='GICP 匹配的最大距离平方（米²）'
    )

    # -------------------------- 配置 small_gicp 节点 --------------------------
    small_gicp_node = Node(
        package='gicp',  # 替换为你的功能包名称（必须和 CMakeLists.txt 中一致）
        executable='gicp_node',  # 编译生成的可执行文件名（和 CMakeLists.txt 中 add_executable 一致）
        name='gicp',  # 节点名称（可自定义，建议和代码中一致）
        output='screen',  # 日志输出到终端（方便调试）
        parameters=[
            # 将启动参数传递给节点的 ROS 参数
            {'map_frame': LaunchConfiguration('map_frame')},
            {'odom_frame': LaunchConfiguration('odom_frame')},
            {'pcd_path': LaunchConfiguration('pcd_path')},
            {'num_threads': LaunchConfiguration('num_threads')},
            {'num_neighbors': LaunchConfiguration('num_neighbors')},
            {'leaf_size': LaunchConfiguration('leaf_size')},
            {'max_dist_sq': LaunchConfiguration('max_dist_sq')}
        ],
        # 可选：如果需要指定节点的命名空间，取消下面注释
        # namespace='relocation',
        # 可选：重启策略（节点崩溃时自动重启）
        # respawn=True,
        # respawn_delay=2.0
    )

    # -------------------------- 组装 LaunchDescription --------------------------
    ld = LaunchDescription()

    # 添加所有声明的启动参数
    ld.add_action(declare_map_frame_arg)
    ld.add_action(declare_odom_frame_arg)
    ld.add_action(declare_pcd_path_arg)
    ld.add_action(declare_num_threads_arg)
    ld.add_action(declare_num_neighbors_arg)
    ld.add_action(declare_leaf_size_arg)
    ld.add_action(declare_max_dist_sq_arg)

    # 添加 small_gicp 节点
    ld.add_action(small_gicp_node)

    return ld