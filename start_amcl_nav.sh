#!/bin/bash
# 启动仅含定位的导航栈（AMCL + Nav2）
# 用于从已有地图进行重定位和导航

source /opt/ros/foxy/setup.bash
cd /home/realmagic/go2_ros2_ws
source install/setup.bash

echo "=========================================="
echo "启动 AMCL 重定位 + Nav2 导航栈"
echo "=========================================="
echo ""
echo "请确保以下前置条件已满足："
echo "  ✅ SLAM Toolbox 已加载地图到 RViz（通过反序列化）"
echo "  ✅ 激光雷达驱动运行中，/scan 话题有数据"
echo "  ✅ go2_base 节点运行中，提供 odom → base_link 变换"
echo ""

# 杀掉旧的进程
echo "清理旧进程..."
pkill -f "nav2_bringup\|amcl\|map_server" 2>/dev/null

sleep 2

echo "启动 AMCL + Map Server + Nav2..."
echo ""

ros2 launch go2_navigation go2_nav2.launch.py

