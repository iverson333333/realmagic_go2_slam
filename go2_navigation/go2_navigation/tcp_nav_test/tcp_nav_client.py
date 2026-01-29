#!/usr/bin/env python3
"""
TCP导航客户端 - 用于测试发送导航目标
发送数据格式: {"x": 1.0, "y": 2.0, "w": 90.0}

模式说明:
  1. 测试模式 (--test): 连接本地服务器(127.0.0.1:8888)进行快速测试
  2. 运行模式 (--run):  连接远程服务器，需要输入IP和端口
  3. 交互模式 (--interactive): 交互式输入目标点和连接信息
"""
import socket
import json
import time
import sys
import argparse

def send_navigation_goal(host='127.0.0.1', port=8888, x=1.0, y=2.0, w=0.0):
    """
    发送导航目标到TCP服务器
    
    Args:
        host: 服务器地址（默认localhost）
        port: 服务器端口（默认8888）
        x: 目标X坐标（米）
        y: 目标Y坐标（米）
        w: 目标朝向（角度制，0-360度）
            0° = 东(X轴正方向)
            90° = 北(Y轴正方向)
            180° = 西(X轴 负方向)
            270° = 南(Y轴负方向)
    """
    # 构建JSON数据
    goal_data = {
        'x': x,
        'y': y,
        'w': w
    }
    
    try:
        # 创建TCP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)  # 10秒超时
        
        print(f'正在连接到 {host}:{port}...')
        sock.connect((host, port))
        print(f'已连接到服务器')
        
        # 发送JSON数据
        json_str = json.dumps(goal_data)
        print(f'发送数据: {json_str}')
        sock.sendall(json_str.encode('utf-8'))
        
        # 接收响应
        response = sock.recv(1024).decode('utf-8')
        print(f'服务器响应: {response}')
        
        response_data = json.loads(response)
        if response_data.get('status') == 'success':
            print(f'✓ 成功: {response_data.get("message")}')
        else:
            print(f'✗ 失败: {response_data.get("message")}')
        
    except socket.timeout:
        print('✗ 连接超时')
    except ConnectionRefusedError:
        print(f'✗ 连接被拒绝，请确保服务器正在运行 ({host}:{port})')
    except Exception as e:
        print(f'✗ 错误: {e}')
    finally:
        sock.close()
        print('连接已关闭\n')

def test_mode():
    """
    测试模式 - 连接本地服务器进行快速测试
    """
    print('=' * 60)
    print('测试模式 - 本地连接测试')
    print('=' * 60)
    print('服务器地址: 127.0.0.1:8888')
    print('使用默认测试数据...\n')
    
    # 发送几个测试点
    test_points = [
        (1.0, 1.0, 0.0, '测试点1: 东方(0°)'),
        (2.0, 2.0, 90.0, '测试点2: 北方(90°)'),
        (1.5, 0.5, 45.0, '测试点3: 东北方(45°)'),
    ]
    
    for x, y, w, desc in test_points:
        print(f'--- {desc} ---')
        send_navigation_goal('127.0.0.1', 8888, x, y, w)
        time.sleep(0.5)
    
    print('\n测试完成！')

def run_mode():
    """
    运行模式 - 连接远程服务器
    需要输入目标服务器的IP地址和端口
    """
    print('=' * 60)
    print('运行模式 - 远程连接')
    print('=' * 60)
    
    # 获取服务器IP
    while True:
        host = input('请输入服务器IP地址 (例: 192.168.1.100): ').strip()
        if host:
            break
        print('IP地址不能为空，请重新输入')
    
    # 获取端口
    while True:
        port_str = input('请输入服务器端口 (默认: 8888): ').strip()
        if not port_str:
            port = 8888
            break
        try:
            port = int(port_str)
            if 1 <= port <= 65535:
                break
            print('端口号必须在1-65535之间')
        except ValueError:
            print('请输入有效的端口号')
    
    print(f'\n目标服务器: {host}:{port}')
    
    # 测试连接
    print('正在测试连接...')
    try:
        test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_sock.settimeout(3.0)
        test_sock.connect((host, port))
        test_sock.close()
        print('✓ 连接成功！\n')
    except Exception as e:
        print(f'✗ 连接失败: {e}')
        print('请检查:')
        print('  1. 服务器IP地址是否正确')
        print('  2. 服务器端口是否正确')
        print('  3. 服务器是否正在运行')
        print('  4. 网络是否连通 (可使用 ping 测试)')
        return
    
    # 循环发送导航点
    print('开始发送导航目标点...')
    print('输入 q 或 quit 退出\n')
    
    count = 0
    while True:
        print(f'\n--- 目标点 #{count + 1} ---')
        
        # 获取X坐标
        x_str = input('X坐标 (米, 输入q退出): ').strip()
        if x_str.lower() in ['q', 'quit']:
            break
        try:
            x = float(x_str)
        except ValueError:
            print('无效的数值，请重新输入')
            continue
        
        # 获取Y坐标
        y_str = input('Y坐标 (米): ').strip()
        try:
            y = float(y_str)
        except ValueError:
            print('无效的数值，请重新输入')
            continue
        
        # 获取方向角
        w_str = input('方向角 (度, 0-360): ').strip()
        try:
            w = float(w_str)
        except ValueError:
            print('无效的数值，请重新输入')
            continue
        
        # 发送目标点
        send_navigation_goal(host, port, x, y, w)
        count += 1
    
    print(f'\n已发送 {count} 个目标点')

def interactive_mode():
    """
    交互模式 - 一次性输入所有信息
    """
    print('=' * 60)
    print('交互模式')
    print('=' * 60)
    
    # 服务器信息
    host = input('服务器IP (默认 127.0.0.1): ').strip() or '127.0.0.1'
    port_str = input('服务器端口 (默认 8888): ').strip()
    port = int(port_str) if port_str else 8888
    
    # 目标点信息
    x = float(input('X坐标 (米): ').strip())
    y = float(input('Y坐标 (米): ').strip())
    w = float(input('方向角 (度): ').strip())
    
    print(f'\n连接到: {host}:{port}')
    print(f'目标点: X={x}, Y={y}, W={w}°\n')
    
    send_navigation_goal(host, port, x, y, w)

def main():
    """
    主函数 - 解析命令行参数
    """
    parser = argparse.ArgumentParser(
        description='TCP导航客户端 - 发送导航目标到远程服务器',
        epilog='''
使用模式:
  
  【测试模式】- 快速本地测试
    python3 tcp_nav_client.py --test
    # 自动连接127.0.0.1:8888，发送3个测试点
  
  【运行模式】- 连接远程服务器
    python3 tcp_nav_client.py --run
    # 会提示输入服务器IP、端口，然后可连续发送多个目标点
  
  【交互模式】- 一次性输入所有信息
    python3 tcp_nav_client.py --interactive
    # 交互式输入服务器信息和目标点
  
  【命令行模式】- 直接指定参数
    python3 tcp_nav_client.py --host 192.168.1.100 --x 2.0 --y 1.5 --w 90
    # 直接发送单个目标点
  
  【批量测试】- 批量发送多个测试点
    python3 tcp_nav_client.py --batch
    # 发送5个预设测试点到本地服务器
    
    python3 tcp_nav_client.py --batch --host 192.168.1.100
    # 发送5个预设测试点到远程服务器

角度说明:
  0°   = 东方 (X轴正方向)
  90°  = 北方 (Y轴正方向)
  180° = 西方 (X轴负方向)
  270° = 南方 (Y轴负方向)
        ''',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # 模式选择
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--test', action='store_true',
                           help='测试模式: 本地快速测试')
    mode_group.add_argument('--run', action='store_true',
                           help='运行模式: 连接远程服务器')
    mode_group.add_argument('--interactive', '-i', action='store_true',
                           help='交互模式: 交互式输入')

    
    # 连接参数
    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help='服务器地址 (默认: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=8888,
                        help='服务器端口 (默认: 8888)')
    
    # 目标点参数
    parser.add_argument('--x', type=float, default=1.0,
                        help='目标X坐标/米 (默认: 1.0)')
    parser.add_argument('--y', type=float, default=2.0,
                        help='目标Y坐标/米 (默认: 2.0)')
    parser.add_argument('--w', type=float, default=0.0,
                        help='目标朝向/度 (0-360, 默认: 0)')
    
    args = parser.parse_args()
    
    # 根据模式执行
    if args.test:
        test_mode()
    
    elif args.run:
        run_mode()
    
    elif args.interactive:
        interactive_mode()
    
    else:
        # 默认: 命令行模式 - 发送单个目标点
        print('=' * 60)
        print(f'命令行模式 - 目标服务器: {args.host}:{args.port}')
        print('=' * 60)
        print(f'=== 发送单个目标点 ===\n')
        send_navigation_goal(args.host, args.port, args.x, args.y, args.w)

if __name__ == '__main__':
    main()