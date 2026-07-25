import socket
import threading
import time

class TaskReceiver:
    def __init__(self, host='0.0.0.0', port=5000):
        """初始化任务接收器
        
        参数:
            host: 监听地址，默认为所有地址
            port: 监听端口，默认为5000
        """
        self.host = host
        self.port = port
        self.server_socket = None
        self.latest_task = None
        self.running = False
        self.clients = []
        
    def start(self):
        """启动任务接收服务器"""
        if self.running:
            print("服务器已经在运行")
            return
            
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 设置端口复用，避免服务器重启时端口被占用
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            print(f"任务接收服务器启动，监听 {self.host}:{self.port}")
            
            # 启动接收线程
            self.receiver_thread = threading.Thread(target=self._accept_connections)
            self.receiver_thread.daemon = True
            self.receiver_thread.start()
            
        except Exception as e:
            print(f"启动服务器失败: {e}")
            self.server_socket.close()
            
    def _accept_connections(self):
        """接受客户端连接的线程函数"""
        while self.running:
            try:
                client_socket, addr = self.server_socket.accept()
                print(f"接受来自 {addr} 的连接")
                
                # 为每个客户端创建一个处理线程
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, addr)
                )
                client_thread.daemon = True
                client_thread.start()
                
                self.clients.append((client_socket, addr, client_thread))
                
            except Exception as e:
                if self.running:  # 只在非正常关闭时打印错误
                    print(f"接受连接时出错: {e}")
    
    def _handle_client(self, client_socket, addr):
        """处理客户端连接的线程函数"""
        try:
            while self.running:
                # 接收数据
                data = client_socket.recv(1024)
                if not data:
                    print(f"客户端 {addr} 断开连接")
                    break
                    
                # 解码并处理任务指令
                task_string = data.decode('utf-8').strip()
                print(f"收到来自 {addr} 的任务指令: {task_string}")
                
                # 验证任务指令格式
                if self._validate_task_string(task_string):
                    self.latest_task = task_string
                    # 发送确认消息
                    client_socket.send("任务接收成功".encode('utf-8'))
                else:
                    # 发送错误消息
                    client_socket.send("任务格式错误".encode('utf-8'))
                    
        except Exception as e:
            print(f"处理客户端 {addr} 时出错: {e}")
        finally:
            client_socket.close()
            # 从客户端列表中移除
            self.clients = [(s, a, t) for s, a, t in self.clients if a != addr]
    
    def _validate_task_string(self, task_string):
        """验证任务指令格式
        
        格式应为: "数字+数字"，例如 "123+321"
        """
        # 检查基本格式
        if '+' not in task_string:
            return False
            
        parts = task_string.split('+')
        if len(parts) != 2:
            return False
            
        # 检查两部分是否都是数字
        if not (parts[0].isdigit() and parts[1].isdigit()):
            return False
            
        # 检查每部分是否是3位数字
        if not (len(parts[0]) == 3 and len(parts[1]) == 3):
            return False
            
        return True
    
    def get_latest_task(self):
        """获取最新的任务指令"""
        return self.latest_task
    
    def stop(self):
        """停止任务接收服务器"""
        self.running = False
        
        # 关闭所有客户端连接
        for client_socket, _, _ in self.clients:
            try:
                client_socket.close()
            except:
                pass
        
        # 关闭服务器socket
        if self.server_socket:
            self.server_socket.close()
            
        print("任务接收服务器已停止")

# 使用示例
# if __name__ == "__main__":
#     receiver = TaskReceiver()
#     receiver.start()
    
#     try:
#         while True:
#             task = receiver.get_latest_task()
#             if task:
#                 print(f"当前任务: {task}")
#             time.sleep(1)
#     except KeyboardInterrupt:
#         print("程序被用户中断")
#     finally:
#         receiver.stop()
