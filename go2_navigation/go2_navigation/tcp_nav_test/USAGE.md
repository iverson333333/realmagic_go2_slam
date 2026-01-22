# TCP客户端使用指南 - 局域网远程连接

## 🎯 四种使用模式

### 1️⃣ 测试模式（推荐先用这个）
**用途**: 快速测试本地连接
```bash
python3 tcp_nav_client.py --test
```
自动连接本地服务器(127.0.0.1:8888)，发送3个测试点。

---

### 2️⃣ 运行模式（局域网使用）⭐
**用途**: 从另一台电脑连接到你的服务器
```bash
python3 tcp_nav_client.py --run
```

**交互式提示**：
```
请输入服务器IP地址 (例: 192.168.1.100): 192.168.1.50
请输入服务器端口 (默认: 8888): 8888

正在测试连接...
✓ 连接成功！

开始发送导航目标点...
输入 q 或 quit 退出

--- 目标点 #1 ---
X坐标 (米, 输入q退出): 2.0
Y坐标 (米): 1.5
方向角 (度, 0-360): 90
正在连接到 192.168.1.50:8888...
已连接到服务器
发送数据: {"x": 2.0, "y": 1.5, "w": 90.0}
服务器响应: {"status": "success", ...}
✓ 成功: 已发布目标点...

--- 目标点 #2 ---
X坐标 (米, 输入q退出): q
```

---

### 3️⃣ 交互模式
**用途**: 一次性输入服务器和目标点信息
```bash
python3 tcp_nav_client.py --interactive
```

---

### 4️⃣ 命令行模式
**用途**: 直接在命令行指定所有参数
```bash
# 连接本地
python3 tcp_nav_client.py --x 2.0 --y 1.5 --w 90

# 连接远程服务器
python3 tcp_nav_client.py --host 192.168.1.100 --x 2.0 --y 1.5 --w 90

# 批量测试（远程）
python3 tcp_nav_client.py --batch --host 192.168.1.100
```

---

## 🌐 局域网连接步骤

### 在服务器电脑（你的电脑）：

**步骤1: 查看IP地址**
```bash
ifconfig
# 或
ip addr show
# 找到类似 192.168.1.50 的IP地址
```

**步骤2: 启动服务端**
```bash
cd ~/go2_ros2_ws
source install/setup.bash
python3 src/go2_ros2_toolbox/go2_navigation/go2_navigation/tcp_nav_test/tcp_nav_server.py
```
显示: `TCP服务器启动成功，监听 0.0.0.0:8888`

**步骤3: 监听话题（可选）**
```bash
# 新终端
source /opt/ros/foxy/setup.bash
ros2 topic echo /goal_pose
```

---

### 在客户端电脑（另一台电脑）：

**步骤1: 复制客户端文件到另一台电脑**
```bash
# 只需要复制tcp_nav_client.py文件
scp tcp_nav_client.py user@remote-pc:~/
```

**步骤2: 测试网络连通性**
```bash
ping 192.168.1.50  # 你的服务器IP
```

**步骤3: 运行客户端**
```bash
# 使用运行模式（推荐）
python3 tcp_nav_client.py --run

# 或直接指定IP
python3 tcp_nav_client.py --host 192.168.1.50 --x 2.0 --y 1.5 --w 90
```

---

## 📊 完整测试流程

### 电脑A（服务器，你的电脑）：
```bash
# 终端1 - 启动服务端
python3 tcp_nav_server.py

# 终端2 - 监听话题
ros2 topic echo /goal_pose
```

### 电脑B（客户端，另一台电脑）：
```bash
# 运行模式
python3 tcp_nav_client.py --run
# 输入电脑A的IP: 192.168.1.50
# 输入端口: 8888
# 然后输入目标点坐标
```

---

## 🔍 故障排查

### 问题1: 连接被拒绝
```bash
# 检查防火墙（服务器端）
sudo ufw status
sudo ufw allow 8888/tcp

# 或临时关闭防火墙测试
sudo ufw disable
```

### 问题2: 无法ping通
```bash
# 确保两台电脑在同一局域网
# 检查是否能互相ping通
ping <对方IP>
```

### 问题3: 找不到服务器
```bash
# 服务器端检查端口监听
netstat -tulpn | grep 8888
# 或
ss -tulpn | grep 8888
```

---

## 📝 使用示例

**快速本地测试：**
```bash
python3 tcp_nav_client.py --test
```

**连接远程服务器：**
```bash
python3 tcp_nav_client.py --run
```

**命令行快速发送：**
```bash
python3 tcp_nav_client.py --host 192.168.1.50 --x 3.0 --y 2.0 --w 45
```

**批量远程测试：**
```bash
python3 tcp_nav_client.py --batch --host 192.168.1.50
```
