import time
import serial
import struct
from Cameracontrol import Cameracontrol
import threading



class Uart:
    def __init__(self, serial_port, baudrate):
        """
        初始化Uart类的实例。

        参数:
            serial_port (str): 串口设备名，例如'/dev/ttyAMA0'
            baudrate (int): 串口通信的波特率，需和与之通信的设备（如STM32）设置相同，默认为115200。
       """    
        self.taskid =None
        self.state = None
        self.page = None    
        self.update_thread = None  # 用于记录当前开启的线程对象
        self.stop_event = None  # 用于控制线程停止的事件对象
        self.uart = serial.Serial(serial_port, baudrate, timeout=0.01)
        self.uart.reset_input_buffer()
        self.uart.reset_output_buffer()
        if not self.uart.isOpen():
            print("无法打开串口")
            raise ValueError("串口打开失败")
        self.running = True  # 用于控制线程的运行状态
        self.lock = threading.Lock()  # 创建一个锁

    def close(self):
        self.uart.close()

    def stm32_ready(self, wait=True):
        self.uart_send_command(task_id=99, param1=0, param2=0, wait=wait, timeout=99999)
        print(f"stm32 is ready")
    def send_order(self,order):
        """
        参数
        order(str)
        """
        num1 = int(order[0:3])
        num2 = int(order[4:7])
        identifier = bytes([0x01])
        end_indentifier = bytes([0xFF])     
        packed_num1 = struct.pack('>H', num1)
        packed_num2 = struct.pack('>H', num2)
        order_to_send = identifier+ packed_num1+ packed_num2+ end_indentifier  #比如123+321对应017b004101ff
        print("order_to_send:",order_to_send.hex())      #打印要发送的任务顺序
        self.uart.write(order_to_send)

    def send_cmd(self, x, y, z, taskid, state, max_retries=3, timeout=0.005):
        """
        发送命令并处理可能的重试
        
        参数:
            x, y, z: 坐标值
            taskid: 任务 ID
            state: 状态值
            max_retries: 最大重试次数
            timeout: 等待错误响应的超时时间
            
        返回:
            bool: 是否成功发送（没有收到错误响应）
        """
        retry_count = 0
        
        while retry_count < max_retries:
            # 清空接收缓冲区，确保不会读取到旧数据
            # self.uart.reset_input_buffer()
            
            # 准备命令数据
            x_rounded = round(x, 3)
            y_rounded = round(y, 3)
            z_rounded = round(z, 3)
            print(f"x: {x_rounded*640:.3f} y: {y_rounded*480:.3f} ")

            # 再乘以1000
            x_val = int(x_rounded * 1000)
            y_val = int(y_rounded * 1000) 
            z_val = int(z_rounded * 1000)
            z_val = z_val + 26946       
            if(z_val <= 0):
                z_val = 0
            if(z_val >= 65535):
                z_val = 65535
                
            identifier = bytes([0x02])
            end_identifier = bytes([0xFF])
            packed_x = struct.pack('>H', x_val)
            packed_y = struct.pack('>H', y_val)
            packed_z = struct.pack('>H', z_val)        
            packed_taskid = struct.pack('>B', taskid)
            packed_state = struct.pack('>B', state)
            cmd_to_send = identifier + packed_x + packed_y + packed_z + packed_taskid + packed_state + end_identifier       
            
            # print("cmd_to_send:", cmd_to_send.hex())
            self.uart.write(cmd_to_send)
            
            # 等待一段时间，看是否收到错误响应
            start_time = time.time()
            error_received = False
            
            while (time.time() - start_time) < timeout:
                if self.uart.in_waiting > 0:
                    response = self.uart.read(self.uart.in_waiting)
                    if 0xFF in response:  # 检查是否收到错误标志
                        print(f"收到错误响应0xFF,重试中...")
                        error_received = True
                        break
                time.sleep(0.01)  # 短暂休眠以避免 CPU 占用过高
            
            # 如果没有收到错误响应，认为发送成功
            if not error_received:
                #stm32正常接收
                return True
            
            # 收到错误响应，重试
            retry_count += 1
            if retry_count < max_retries:
                print(f"重试发送命令 ({retry_count}/{max_retries})...")
                time.sleep(0.1)  # 短暂延时后重试
        
        print(f"发送命令失败，已重试 {max_retries} 次")
        return False
    def update_color_center2stm(self, task_id, color_id, cam:Cameracontrol):
        while True:
            if self.stop_event and self.stop_event.is_set():  # 检查停止事件是否被设置
                break
            try:
                x, y= cam.update_color_center(color_id)
                z = 0#stm32使用陀螺仪的数据

                if((x!=None)&(z!=None)):
                    self.send_cmd(x, y, z, task_id, 0)

                # self.send_cmd2screen(x,y,0,task_id,0)
                # time.sleep(0.5)
            except Exception as e:
                print(f"更新坐标线程发生错误: {e}")
                continue

    def start_update(self, task_id, color_id, cam:Cameracontrol):
        """
        开启一个线程不断获取指定颜色的坐标并发给stm32(根据taskid不同发送的taskid也不同)
        参数：
            当下任务，目标颜色
        """
        if self.update_thread and self.update_thread.is_alive():
            self.stop_update()  # 如果已有线程在运行且未结束，先停止它

        self.stop_event = threading.Event()  # 创建停止事件对象
        update_thread = threading.Thread(target=self.update_color_center2stm, args=(task_id, color_id, cam))
        update_thread.start()
        print("线程开启")
        self.update_thread = update_thread  # 记录当前开启的线程
    def stop_update(self):
        """
        停止当前开启的线程
        """
        print("线程结束")
        if self.stop_event:
            self.stop_event.set()  # 设置停止事件，让线程内循环结束
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join()  # 等待线程结束
            self.update_thread = None  # 重置线程记录
            self.stop_event = None  # 重置停止事件对象

    def send_order2screen(self,order):
        """
        发送任务顺序到串口屏
        参数：
            字符串任务顺序如"123+321"
        """
        if(self.page!="page 0"):
            self.page ="page 0"
            self.uart.write(self.page.encode('utf-8'))
            self.uart.write(bytes.fromhex('ff ff ff'))

        order_to_send = "page0.t0.txt=" + "\"" + order + "\""
        self.uart.write(order_to_send.encode('utf-8'))
        self.uart.write(bytes.fromhex('ff ff ff'))    

    def send_cmd2screen(self, x, y, z, taskid, state):
        """
        发送任务状态到串口屏
        xyz均为三位小数
        """
        x = round(x, 3)
        y = round(y, 3)
        if(self.page!="page 1"):
            self.page ="page 1"
            self.uart.write(self.page.encode('utf-8'))
            self.uart.write(bytes.fromhex('ff ff ff'))

        x_to_send = "page1.t1.txt="+ "\"" +str(x)+ "\""
        self.uart.write(x_to_send.encode('utf-8'))
        self.uart.write(bytes.fromhex('ff ff ff'))

        y_to_send = "page1.t3.txt="+ "\"" +str(y)+ "\""
        self.uart.write(y_to_send.encode('utf-8'))
        self.uart.write(bytes.fromhex('ff ff ff'))

        z_to_send = "page1.t5.txt="+ "\"" +str(z)+ "\""
        self.uart.write(z_to_send.encode('utf-8'))
        self.uart.write(bytes.fromhex('ff ff ff'))

        taskid_to_send = "page1.t7.txt="+ "\"" +str(taskid)+ "\""
        self.uart.write(taskid_to_send.encode('utf-8'))
        self.uart.write(bytes.fromhex('ff ff ff'))

        state_to_send = "page1.t9.txt="+ "\"" +str(state)+ "\""
        self.uart.write(state_to_send.encode('utf-8'))
        self.uart.write(bytes.fromhex('ff ff ff'))

    
    def wait_for_signal(self, target_taskid, target_state, timeout=9999, success_message=""):
        """
        等待并验证特定的串口信号，带超时功能
        
        参数:
        - target_taskid: 期望的任务ID
        - target_state: 期望的状态值
        - timeout: 超时时间（秒），默认9999秒
        - success_message: 成功时打印的消息（可选）
        
        返回:
        - bool: 是否接收到预期的信号
        """
        start_time = time.time()
        
        while True:
            # 检查是否超时
            if time.time() - start_time > timeout:
                print(f"等待信号超时！目标taskid={target_taskid}, state={target_state}")
                return False
                
            if self.taskid == target_taskid and self.state == target_state:
                if success_message:
                    print(success_message)
                return True
                    
            # 短暂休眠以避免CPU占用过高
            time.sleep(0.01)

    def start_receive_stm32(self):
        # 创建并启动接收线程
        receive_thread = threading.Thread(target=self.receive_data)
        receive_thread.daemon = True  # 设置为守护线程
        receive_thread.start()

    def receive_data(self):
        while self.running:
            if self.uart.in_waiting >= 2:
                get_from_stm = self.uart.read(2)
                with self.lock:  # 使用锁保护对全局变量的访问
                    self.taskid = get_from_stm[0]
                    self.state = get_from_stm[1]
                # print('Received:', self.taskid, self.state)

            time.sleep(0.04)  # 添加短暂延时，避免过于频繁的读取

    def stop_receiving(self):
        self.running = False  # 停止接收线程
        self.uart.close()  # 关闭串口

#



