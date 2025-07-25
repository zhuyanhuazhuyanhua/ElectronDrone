#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket
import threading

# 全局变量，保存最新的字符串消息
latest_message = "No message received"

# ROS 话题回调函数
def chatter_callback(msg):
    global latest_message
    latest_message = msg

# TCP 客户端处理线程
def handle_client(conn, addr):
    rospy.loginfo(f"Connected by {addr}")
    try:
        data = conn.recv(1024)  # 接收客户端请求（内容可忽略）
        if data:
            msg_to_send = latest_message 
            response = msg_to_send.encode('utf-8')
            conn.sendall(response)
    except Exception as e:
        rospy.logwarn(f"Error handling client {addr}: {e}")
    finally:
        conn.close()

# 自动获取本机的局域网 IP 地址
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '192.168.0.4'
    finally:
        s.close()
    return ip

# TCP服务器主函数
def tcp_server(host='0.0.0.0', port=8888):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    rospy.loginfo(f"TCP server listening on {host}:{port}")
    
    while not rospy.is_shutdown():
        try:
            conn, addr = server_socket.accept()
            threading.Thread(target=handle_client, args=(conn, addr)).start()
        except Exception as e:
            rospy.logerr(f"Error accepting connection: {e}")
            break
    server_socket.close()

if __name__ == '__main__':
    rospy.init_node('tcp_ros_server')

    # 订阅 ROS 字符串话题
    rospy.Subscriber('/fire_content', String, chatter_callback)

    # 获取本机 IP 并启动 TCP 服务器
    local_ip = get_local_ip()
    rospy.loginfo(f"Binding TCP server to local IP: {local_ip}")
    tcp_thread = threading.Thread(target=tcp_server, args=(local_ip,))
    tcp_thread.daemon = True
    tcp_thread.start()

    # 保持 ROS 节点运行
    rospy.spin()
