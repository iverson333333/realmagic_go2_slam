from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包目录
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    go2_slam_dir = get_package_share_directory('go2_slam')
    
    # 声明参数
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: mapping or localization'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(go2_slam_dir, '../../../user_map/my_map'),
        description='Path to the map file (without extension)'
    )

    # 选择配置文件
    config_file = os.path.join(go2_slam_dir, 'config', 'mapper_params_online_async.yaml')

    # slam_toolbox 节点
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_file]
    )

    ld = LaunchDescription([
        mode_arg,
        map_file_arg,
        slam_node
    ])
    
    return ld
