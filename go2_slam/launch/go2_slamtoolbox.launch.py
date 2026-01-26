from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明模式参数
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='SLAM mode: mapping or localization'
    )
    
    mode = LaunchConfiguration('mode')
    
    # 获取slam_toolbox包的路径
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    go2_slam_dir = get_package_share_directory('go2_slam')
    
    # 根据模式选择配置文件
    # mapping 模式使用 mapper_params_online_async.yaml
    # localization 模式使用 mapper_params_localization.yaml
    config_mapping = os.path.join(go2_slam_dir, 'config', 'mapper_params_online_async.yaml')
    config_localization = os.path.join(go2_slam_dir, 'config', 'mapper_params_localization.yaml')
    
    # 使用 PythonExpression 根据模式选择配置文件
    slam_config = PythonExpression([
        "'", config_localization, "' if '", mode, "' == 'localization' else '", config_mapping, "'"
    ])

    # 包含slam_toolbox的launch文件
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'params_file': slam_config,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        mode_arg,
        slam_toolbox_launch
    ])