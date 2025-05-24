import socket

def tcp_client(server_ip, port=8888):
    try:
        # 创建 TCP socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # 连接到服务器
        client_socket.connect((server_ip, port))
        print(f"Connected to server {server_ip}:{port}")
        
        # 发送任意请求内容（可为空）
        client_socket.sendall(b'get_message')

        # 接收响应数据
        data = client_socket.recv(1024)
        print("Received from server:", data.decode('utf-8'))

    except Exception as e:
        print(f"Connection failed: {e}")
    finally:
        client_socket.close()

if __name__ == '__main__':
    # 替换为服务端实际 IP（可从服务端日志或 ifconfig 查看）
    server_ip = '10.21.190.49'  # ← 修改为你服务端的 IP
    tcp_client(server_ip)
