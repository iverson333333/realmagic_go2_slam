#!/bin/bash
# AMCL 诊断脚本

echo "=========================================="
echo "      AMCL 和定位系统诊断"
echo "=========================================="
echo ""

# 1. 检查节点是否运行
echo "1️⃣  检查关键节点状态："
echo "-----------------------------------------"
source /opt/ros/foxy/setup.bash

echo "AMCL 节点："
ros2 node list 2>/dev/null | grep -i amcl || echo "  ❌ AMCL 未启动"

echo ""
echo "Map Server 节点："
ros2 node list 2>/dev/null | grep -i map_server || echo "  ❌ Map Server 未启动"

echo ""
echo "2️⃣  检查关键话题："
echo "-----------------------------------------"
echo "扫描话题 (/scan)："
ros2 topic list 2>/dev/null | grep scan || echo "  ❌ /scan 话题不存在"

echo ""
echo "AMCL 位置估计 (/amcl_pose)："
ros2 topic list 2>/dev/null | grep amcl_pose || echo "  ❌ /amcl_pose 话题不存在"

echo ""
echo "3️⃣  检查扫描数据是否有效："
echo "-----------------------------------------"
echo "获取最新扫描数据（3秒）..."
timeout 3 ros2 topic echo /scan 2>/dev/null | head -20 || echo "  ⚠️  无扫描数据或超时"

echo ""
echo "4️⃣  检查 AMCL 参数："
echo "-----------------------------------------"
echo "AMCL 参数列表："
ros2 param list /amcl 2>/dev/null | head -10 || echo "  ❌ 无法获取 AMCL 参数"

echo ""
echo "5️⃣  检查坐标变换："
echo "-----------------------------------------"
echo "显示 tf 树（最多显示10行）："
ros2 run tf2_tools view_frames 2>/dev/null || echo "  ⚠️  无法获取 tf 信息"

echo ""
echo "6️⃣  实时 AMCL 位置估计："
echo "-----------------------------------------"
echo "监听 /amcl_pose 话题（3秒）..."
timeout 3 ros2 topic echo /amcl_pose 2>/dev/null || echo "  ❌ 未收到位置估计"

echo ""
echo "=========================================="
echo "诊断完成"
echo "=========================================="
echo ""
echo "💡 常见问题排查："
echo "  1. 如果 AMCL 未启动：检查 go2_nav2.launch.py 是否正确包含 amcl_launch"
echo "  2. 如果 /scan 话题不存在：检查激光雷达驱动是否启动"
echo "  3. 如果 /amcl_pose 没有更新：检查 AMCL 参数和扫描数据质量"
echo "  4. 在 RViz 中使用 '2D Pose Estimate' 给 AMCL 初始位置"
