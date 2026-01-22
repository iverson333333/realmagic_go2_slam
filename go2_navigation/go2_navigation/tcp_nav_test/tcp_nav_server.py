#!/usr/bin/env python3
"""
TCP导航服务端 - 接收JSON格式的导航目标并发布到/goal_pose话题
接收数据格式: {"x": 1.0, "y": 2.0, "w": 90.0}
其中w为角度制（0-360度）
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import json
import threading
import math

class TcpNavServer(Node):
    def __init__(self):
        super().__init__('tcp_nav_server')
        
        # 声明参数
        self.declare_parameter('host', '0.0.0.0')  # 监听所有网络接口
        self.declare_parameter('port', 8888)
        
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        
        # 创建/goal_pose发布器
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 创建TCP服务器
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.get_logger().info(f'TCP服务器启动成功，监听 {self.host}:{self.port}')
        except Exception as e:
            self.get_logger().error(f'TCP服务器启动失败: {e}')
            return
        
        # 启动TCP服务器线程
        self.tcp_thread = threading.Thread(target=self.tcp_server_loop, daemon=True)
        self.tcp_thread.start()
    
    def tcp_server_loop(self):
        """TCP服务器主循环"""
        while rclpy.ok():
            try:
                self.get_logger().info('等待客户端连接...')
                client_socket, client_address = self.server_socket.accept()
                self.get_logger().info(f'客户端已连接: {client_address}')
                
                # 处理客户端连接
                self.handle_client(client_socket, client_address)
                
            except Exception as e:
                self.get_logger().error(f'服务器循环错误: {e}')
    
    def handle_client(self, client_socket, client_address):
        """处理单个客户端连接"""
        try:
            while rclpy.ok():
                # 接收数据（最多1024字节）
                data = client_socket.recv(1024)
                
                if not data:
                    self.get_logger().info(f'客户端 {client_address} 断开连接')
                    break
                
                try:
                    # 解析JSON数据
                    json_str = data.decode('utf-8').strip()
                    self.get_logger().info(f'接收到数据: {json_str}')
                    
                    goal_data = json.loads(json_str)
                    
                    # 提取x, y, w
                    x = float(goal_data.get('x', 0.0))
                    y = float(goal_data.get('y', 0.0))
                    w_deg = float(goal_data.get('w', 0.0))  # 角度制
                    
                    # 发布到/goal_pose
                    self.publish_goal(x, y, w_deg)
                    
                    # 发送响应
                    response = json.dumps({
                        'status': 'success',
                        'message': f'已发布目标点: x={x}, y={y}, w={w_deg}度'
                    })
                    client_socket.sendall(response.encode('utf-8'))
                    
                except json.JSONDecodeError as e:
                    self.get_logger().error(f'JSON解析错误: {e}')
                    error_response = json.dumps({
                        'status': 'error',
                        'message': f'JSON解析失败: {str(e)}'
                    })
                    client_socket.sendall(error_response.encode('utf-8'))
                    
                except Exception as e:
                    self.get_logger().error(f'处理数据错误: {e}')
                    error_response = json.dumps({
                        'status': 'error',
                        'message': f'处理失败: {str(e)}'
                    })
                    client_socket.sendall(error_response.encode('utf-8'))
                    
        except Exception as e:
            self.get_logger().error(f'客户端处理错误: {e}')
        finally:
            client_socket.close()
            self.get_logger().info(f'客户端 {client_address} 连接已关闭')
    
    def publish_goal(self, x, y, w_deg):
        """
        发布目标点到/goal_pose话题
        
        Args:
            x: X坐标（米）
            y: Y坐标（米）
            w_deg: 朝向角度（角度制，0-360）
        """
        # 将角度转换为弧度
        w_rad = math.radians(w_deg)
        
        # 将欧拉角转换为四元数
        # yaw (Z轴旋转) 转四元数
        qz = math.sin(w_rad / 2.0)
        qw = math.cos(w_rad / 2.0)
        
        # 创建PoseStamped消息
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'  # 使用map坐标系
        
        # 设置位置
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        
        # 设置方向（四元数）
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw
        
        # 发布消息
        self.goal_publisher.publish(goal_msg)
        
        self.get_logger().info(
            f'已发布目标点到/goal_pose: '
            f'x={x:.2f}m, y={y:.2f}m, '
            f'角度={w_deg:.1f}° (弧度={w_rad:.3f}, qz={qz:.3f}, qw={qw:.3f})'
        )
    
    def destroy_node(self):
        """节点销毁时关闭TCP服务器"""
        try:
            self.server_socket.close()
            self.get_logger().info('TCP服务器已关闭')
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TcpNavServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
