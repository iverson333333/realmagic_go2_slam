# TCP导航测试系统

通过TCP接收JSON格式的导航目标并发布到ROS2的`/goal_pose`话题。

## 📁 文件说明

- `tcp_nav_server.py` - TCP服务端（ROS2节点），接收JSON并发布到/goal_pose
- `tcp_nav_client.py` - TCP客户端，用于测试发送导航目标
- `__init__.py` - Python包初始化文件

## 📊 数据格式

### 发送格式（JSON）
```json
{
  "x": 1.0,    // X坐标（米）
  "y": 2.0,    // Y坐标（米）
  "w": 90.0    // 朝向角度（度，0-360）
}
```

## 🚀 使用方法

### 1. 启动TCP服务端（ROS2节点）

```bash
# 终端1：确保已source环境
cd ~/go2_ros2_ws
source install/setup.bash

# 运行TCP服务端
python3 src/go2_ros2_toolbox/go2_navigation/go2_navigation/tcp_nav_test/tcp_nav_server.py
```

服务端默认监听 `0.0.0.0:8888`（所有网络接口）

### 2. 测试发送（本地）

**单个目标点：**
```bash
# 终端2
cd ~/go2_ros2_ws/src/go2_ros2_toolbox/go2_navigation/go2_navigation/tcp_nav_test

# 发送目标点 (1.0, 2.0)，朝向北(90度)
python3 tcp_nav_client.py --x 1.0 --y 2.0 --w 90

# 发送目标点 (2.5, 3.5)，朝向东北(45度)
python3 tcp_nav_client.py --x 2.5 --y 3.5 --w 45
```

**批量测试：**
```bash
python3 tcp_nav_client.py --batch
```

### 3. 局域网发送（从其他电脑）

**在其他电脑上：**
```bash
# 替换为服务端实际IP地址
python3 tcp_nav_client.py --host 192.168.1.100 --x 2.0 --y 1.5 --w 90
```

## 📝 验证消息发布

**查看/goal_pose话题：**
```bash
# 终端3
source /opt/ros/foxy/setup.bash
ros2 topic echo /goal_pose
```

**输出示例：**
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: map
pose:
  position:
    x: 1.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.7071067811865475  # sin(90°/2)
    w: 0.7071067811865476  # cos(90°/2)
```

## 🔧 自定义端口

```bash
# 服务端使用9999端口
python3 tcp_nav_server.py --ros-args -p port:=9999

# 客户端连接9999端口
python3 tcp_nav_client.py --port 9999 --x 1.0 --y 1.0 --w 0
```

## 💻 Python代码示例

从其他程序发送导航目标：

```python
import socket
import json

def send_goal(host, port, x, y, w):
    goal_data = {"x": x, "y": y, "w": w}
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    sock.sendall(json.dumps(goal_data).encode('utf-8'))
    
    response = sock.recv(1024).decode('utf-8')
    print(response)
    sock.close()

# 使用示例
send_goal('192.168.1.100', 8888, 2.0, 3.0, 90.0)
```

## ⚠️ 注意事项

1. **坐标系**：默认使用`map`坐标系
2. **角度范围**：输入0-360度，程序会自动转换为四元数
3. **网络防火墙**：确保8888端口未被防火墙阻止
4. **IP地址**：局域网通信时使用实际IP（通过`ifconfig`或`ip addr`查看）

## 🐛 故障排查

**连接被拒绝：**
```bash
# 检查服务端是否运行
ps aux | grep tcp_nav_server

# 检查端口占用
netstat -tulpn | grep 8888
```

**找不到主机：**
```bash
# 检查网络连通性
ping 192.168.1.100
```
