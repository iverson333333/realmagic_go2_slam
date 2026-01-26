from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # 获取配置文件路径
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(
        get_package_share_directory('go2_navigation'),
        'config',
        'nav2_params.yaml'
    )

    # 包含 Nav2 的完整启动文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'false',
            'use_respawn': 'false',
        }.items(),
    )

    ld.add_action(nav2_launch)

    return ld
