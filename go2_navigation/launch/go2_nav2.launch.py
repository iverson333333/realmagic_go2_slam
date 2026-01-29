from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 创建LaunchDescription对象
    ld = LaunchDescription()

    # 获取包路径
    nav2_dir = get_package_share_directory('go2_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 设置配置文件路径
    nav2_config = os.path.join(nav2_dir, 'config', 'nav2_params.yaml')

    # 1. 启动 AMCL 和 Map Server（用于重定位）
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_dir, 'launch', 'amcl_launch.py')
        ])
    )

    # 2. 包含nav2的导航启动文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_config,
            'use_sim_time': 'false',
            'autostart': 'true',
            'map': '',  # 留空，因为我们使用SLAM的地图而不是静态地图
        }.items(),
    )

    # 添加launch动作
    #ld.add_action(amcl_launch)
    ld.add_action(nav2_launch)

    return ld