#!/usr/bin/env python3
"""
TCP导航客户端 - 用于测试发送导航目标
发送数据格式: {"x": 1.0, "y": 2.0, "w": 90.0}
"""
import socket
import json
import time

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
            180° = 西(X轴负方向)
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

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='TCP导航客户端 - 发送导航目标点',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例用法:
  # 发送目标点 (1.0, 2.0)，朝向东(0度)
  python3 tcp_nav_client.py --x 1.0 --y 2.0 --w 0
  
  # 发送目标点 (2.5, 3.5)，朝向北(90度)
  python3 tcp_nav_client.py --x 2.5 --y 3.5 --w 90
  
  # 连接远程服务器
  python3 tcp_nav_client.py --host 192.168.1.100 --x 1.0 --y 1.0 --w 45
  
  # 批量发送多个目标点
  python3 tcp_nav_client.py --batch
        '''
    )
    
    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help='服务器地址 (默认: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=8888,
                        help='服务器端口 (默认: 8888)')
    parser.add_argument('--x', type=float, default=1.0,
                        help='目标X坐标/米 (默认: 1.0)')
    parser.add_argument('--y', type=float, default=2.0,
                        help='目标Y坐标/米 (默认: 2.0)')
    parser.add_argument('--w', type=float, default=0.0,
                        help='目标朝向/度 (0-360, 默认: 0)')
    parser.add_argument('--batch', action='store_true',
                        help='批量发送测试数据')
    
    args = parser.parse_args()
    
    if args.batch:
        print('=== 批量测试模式 ===\n')
        # 测试多个目标点
        test_goals = [
            (1.0, 0.0, 0.0, '点1: 东方1米'),
            (0.0, 1.0, 90.0, '点2: 北方1米'),
            (-1.0, 0.0, 180.0, '点3: 西方1米'),
            (0.0, -1.0, 270.0, '点4: 南方1米'),
            (2.0, 2.0, 45.0, '点5: 东北方'),
        ]
        
        for x, y, w, desc in test_goals:
            print(f'--- {desc} ---')
            send_navigation_goal(args.host, args.port, x, y, w)
            time.sleep(1)  # 间隔1秒
    else:
        # 单个目标点
        print('=== 发送单个目标点 ===\n')
        send_navigation_goal(args.host, args.port, args.x, args.y, args.w)

if __name__ == '__main__':
    main()
