from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # 配置文件路径
    nav2_params = os.path.join(
        get_package_share_directory('go2_navigation'),
        'config',
        'nav2_params.yaml'
    )

    # =========== AMCL 节点 ===========
    # AMCL是用来重定位的关键节点
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('amcl/pose', '/amcl_pose'),
        ]
    )

    # =========== Lifecycle Manager ===========
    # 管理 AMCL 的生命周期
    # 注意：不启动 map_server，因为 SLAM Toolbox 已经在发布 /map 话题
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['amcl']}  # 只管理 AMCL，不包含 map_server
        ]
    )

    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_localization)

    return ld
