# 地图加载 + 重定位 + 导航完整指南

## 核心概念

### 1. SLAM Toolbox 的两种模式

#### Mapping 模式（建图）
```yaml
mode: mapping
map_start_pose: [0.0, 0.0, 0.0]
```
- 实时构建地图
- 不加载预存地图
- 适合首次探索环境

#### Localization 模式（定位）
```yaml
mode: localization
map_file_name: /path/to/map/base_name  # 不要加 .posegraph 或 .data
```
- 加载已有地图
- **只进行定位，不更新地图**
- 适合已知环境的导航
- 地图文件必须存在 `.posegraph` 和 `.data` 两个文件

### 2. 重定位的工作原理

**AMCL（自适应蒙特卡洛定位）**是重定位的关键：

```
机器人 --(扫描数据)--> AMCL --(与地图对齐)--> 粒子滤波 --(估计位置)--> Pose
```

**步骤：**
1. 机器人在地图上的位置未知
2. 点击 RViz 的 "2D Pose Estimate" 按钮
3. 在地图上标记初始位置和朝向
4. AMCL 接收初始估计，使用激光扫描对齐地图
5. 粒子快速收敛，定位成功

**影响重定位的关键参数：**
```yaml
max_particles: 3000          # 更多粒子 → 更容易找到正确位置，但CPU成本高
recovery_alpha_fast: 0.001   # 更快的权重衰减 → 更快收敛
recovery_alpha_slow: 0.001
laser_likelihood_max_dist: 2.0  # 允许的最大扫描误差
```

### 3. 完整的工作流

```
[建图阶段]
┌─────────────────────────────────────┐
│ 1. SLAM Mapping 模式建图             │
│    手动控制机器人探索环境           │
│ 2. 序列化并保存地图                  │
│    生成 my_map.posegraph + my_map.data
│ 3. 导出为 ROS 标准格式               │
│    生成 my_map.yaml + my_map.pgm     │
└─────────────────────────────────────┘

[导航阶段]
┌─────────────────────────────────────┐
│ 1. SLAM Localization 模式加载地图    │
│    + Map Server 提供 .yaml 格式地图
│    + AMCL 进行重定位                 │
│ 2. Nav2 路径规划和导航               │
│    + GlobalPlanner 全局规划          │
│    + LocalPlanner 局部避障           │
│ 3. 用户输入导航目标                  │
│    机器人自动到达                    │
└─────────────────────────────────────┘
```

## 文件格式说明

### SLAM Toolbox 地图格式
- `my_map.posegraph` - 位姿图（SLAM内部格式）
- `my_map.data` - 栅格地图数据（SLAM内部格式）

### ROS 标准地图格式
```yaml
# my_map.yaml
image: my_map.pgm           # 地图图像文件
resolution: 0.05            # 每个像素代表0.05米
origin: [0.0, 0.0, 0.0]    # 地图原点
occupied_thresh: 0.65
free_thresh: 0.25
negate: 0
```

## 常见问题

### Q: 为什么 localization 模式没加载地图？
**A:** 检查以下几点：
1. 地图文件完整性：
   ```bash
   ls -la /path/to/my_map.*
   # 应该看到 .posegraph 和 .data
   ```
2. `map_file_name` 不要包含扩展名
3. YAML 缩进正确
4. 文件权限可读

### Q: 为什么 AMCL 无法重定位？
**A:** 
1. 初始位置估计误差过大（超过地图范围）
2. 激光扫描与地图不匹配（环境改变太多）
3. 粒子数不足（增加 max_particles）
4. 机器人移动太快，扫描匹配失败

### Q: SLAM Toolbox 占用CPU过高？
**A:** 在 mapping 模式中：
```yaml
minimum_travel_distance: 1.0   # 增加两次扫描的最小距离
minimum_travel_heading: 0.5    # 增加最小转角
```

## 调试命令

```bash
# 查看SLAM状态
ros2 topic echo /scan
ros2 topic echo /slam_toolbox/scan_matched_points2

# 查看定位状态
ros2 topic echo /amcl_pose              # AMCL 估计的位置
ros2 topic echo /tf                      # 坐标变换
ros2 topic echo /particlecloud           # 粒子分布（调试用）

# 查看导航状态
ros2 action list                         # 导航动作
ros2 topic echo /tf_static               # 静态坐标变换

# 参数验证
ros2 param list /slam_toolbox
ros2 param get /slam_toolbox mode
```

## 最后提醒

1. **建图和定位不能同时进行** - 在 `mapper_params_online_async.yaml` 中只能选择一种模式
2. **地图文件必须存在** - 定位模式需要 `.posegraph` 和 `.data` 文件
3. **初始位置估计很关键** - 尽量使初始姿态接近真实位置
4. **激光范围检查** - 确保 `/scan` 话题正确发布且有效数据
