from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取包目录
    pkg_dir = FindPackageShare('fiesta').find('fiesta')
    
    # YAML参数文件路径
    params_file = os.path.join(pkg_dir, 'config', 'fiesta_params.yaml')
    
    # 创建节点
    fiesta_node = Node(
        package='fiesta',
        # 注意：根据您的C++代码，这个可执行文件可能是不同的
        # 请根据实际情况调整
        executable='map_node',
        name='fiesta',
        output='screen',
        parameters=[params_file],
    )
    
    return LaunchDescription([
        fiesta_node
    ])