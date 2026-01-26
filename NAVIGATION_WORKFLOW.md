#!/bin/bash
# go2_ros2 完整导航工作流

# ==========================================
# 步骤1: 建图（首次或需要更新地图时）
# ==========================================
echo "=== 步骤1: 启动建图模式 ==="
echo "运行以下命令启动建图："
echo "ros2 launch go2_slam go2_slamtoolbox.launch.py"
echo ""
echo "然后用以下命令手动控制机器人绕行探索环境："
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel"
echo ""
echo "建图完成后，在RViz中保存地图："
echo "  1. 在 Rviz 的 SlamToolbox 插件中找到 'Serialize Map' 按钮"
echo "  2. 输入地图名称: /home/realmagic/go2_ros2_ws/src/go2_ros2_toolbox/user_map/
echo "  3. 这会生成 my_map.posegraph 和 my_map.data 文件"
echo ""

# ==========================================
# 步骤2: 导出为 ROS 标准地图格式
# ==========================================
echo "=== 步骤2: 导出地图为标准格式 ==="
echo "运行以下命令将 SLAM 地图导出为 .pgm + .yaml："
echo "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"name: { data: '/home/realmagic/go2_ros2_ws/src/go2_ros2_toolbox/user_map/my_map' }\""
echo ""

# ==========================================
# 步骤3: 启动定位和导航
# ==========================================
echo "=== 步骤3: 启动定位和导航模式 ==="
echo "打开4个终端执行以下命令："
echo ""
echo "终端1 - 启动SLAM定位节点："
echo "  source /opt/ros/foxy/setup.bash"
echo "  cd /home/realmagic/go2_ros2_ws && source install/setup.bash"
echo "  ros2 launch go2_slam go2_slamtoolbox_localization.launch.py"
echo ""
echo "终端2 - 启动Nav2导航栈："
echo "  source /opt/ros/foxy/setup.bash"
echo "  cd /home/realmagic/go2_ros2_ws && source install/setup.bash"
echo "  ros2 launch go2_navigation go2_nav2.launch.py"
echo ""
echo "终端3 - 启动 RViz 可视化："
echo "  source /opt/ros/foxy/setup.bash"
echo "  cd /home/realmagic/go2_ros2_ws && source install/setup.bash"
echo "  rviz2 -d /path/to/your/config.rviz"
echo ""
echo "终端4 - 发送重定位初始姿态估计："
echo "  source /opt/ros/foxy/setup.bash"
echo "  cd /home/realmagic/go2_ros2_ws && source install/setup.bash"
echo ""
echo "在 RViz 中："
echo "  1. 点击 '2D Pose Estimate' 按钮"
echo "  2. 在地图上点击机器人的初始位置，并通过鼠标拖拽设置朝向"
echo "  3. AMCL 会自动对齐激光扫描与地图，实现重定位"
echo ""
echo "重定位成功后，可以设置导航目标："
echo "  1. 点击 '2D Nav Goal' 按钮"
echo "  2. 在地图上点击目标位置，设置朝向"
echo "  3. 机器人会自动规划路径并导航到目标"
echo ""

# ==========================================
# 步骤4: 验证
# ==========================================
echo "=== 步骤4: 验证系统状态 ==="
echo "运行以下命令验证各节点和参数："
echo ""
echo "检查节点是否运行："
echo "  ros2 node list"
echo ""
echo "检查 AMCL 定位精度："
echo "  ros2 topic echo /amcl_pose"
echo ""
echo "检查导航目标状态："
echo "  ros2 action list"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}\""
echo ""
