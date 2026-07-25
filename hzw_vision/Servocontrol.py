import serial
import time


#旋转轴-----Filter舵机1---------------------------- 
ANGLE_CENTER=2048#正对前方(即首先在中间位置进行中文校准)
ANLGE_STORE=3581#正对储物盘
ANGLE_HALF=2839
#升降----Filter舵机2----从最低到最高对应2048-5900----------
HEIGHT_MIN=2048
HEIGHT_MAX=6100
HEIGHT_1=2088 #物料放地面高度
HEIGHT_2=3730 #码垛高度
HEIGHT_3=3860 #物料盘抓取高度(detect)
HEIGHT_4=5050#物料后抬升高度
HEIGHT_5=5300 #物料储存后抬升高度
HEIGHT_6=4450 #抓取储存的物料高度
HEIGHT_7=5780  #blog识别圆环高度#5650
HEIGHT_8=2248 #夹住物料精调高度
HEIGHT_9=4140 #转盘圆环精调高度
#夹爪-----PWM舵机3---------------------------
LINE_OPEN=86#180du da kai
OPEN=56 #夹爪打开
HALF=51 #夹爪半张开
HOLD=29 #夹爪闭合
#储物盘-----PWM舵机9--------------------
STORE_1=3#红色盘
STORE_2=79 #绿色盘
STORE_3=152 #蓝色盘

class Servocontrol:
    def __init__(self):
        # 配置PWM舵机串口
        self.pwm_ser = serial.Serial("/dev/ttyAMA1", 9600)
        # 配置Filter舵机串口
        self.filter_ser = serial.Serial("/dev/ttyAMA3", 115200)
        
        self.cnt1 = 0
        self.cnt2 = 0
        self.cnt3 = 0
        # 用于记录每个舵机上一次设置的角度
        self.last_angle_dict = {}
        self.step_size_dict = {
            16: 5,  # 夹爪舵机
            9: 5     # 储物盘舵机
        }
        # 进行机械臂初始化操作
        # self.initialize_arm()
                       
    def PWMServo(self, servonum, target_angle):
        """PWM舵机控制，仅用于夹爪(舵机3)和储物盘(舵机9)"""
        if servonum < 1 or servonum > 16:
            raise ValueError("servonum_error")
        if target_angle < 0 or target_angle > 180:
            raise ValueError("target_angle_error")
        servonum = 64 + servonum
        step_size = self.step_size_dict.get(servonum, 1)
        # 如果是第一次设置该舵机角度，初始化为目标角度，否则用上一次角度
        current_angle = self.last_angle_dict.get(servonum, target_angle)
        self.last_angle_dict[servonum] = target_angle
        # 判断转动方向，确定步进步数
        step_count = int(abs(target_angle - current_angle) / step_size)
        step_direction = 1 if target_angle > current_angle else -1
        # for _ in range(step_count):
        #     current_angle += step_size * step_direction
        #     date1 = int(current_angle / 100 + 48)
        #     date2 = int((current_angle % 100) / 10 + 48)
        #     date3 = int(current_angle % 10 + 48)
        #     cmd = bytearray([36, servonum, date1, date2, date3, 35])
        #     self.pwm_ser.write(cmd)
        #     time.sleep(0.008)  # 适当缩短每次步进的间隔时间，可调整
        # 最后再设置一次目标角度，确保最终能准确到达
        date1 = int(target_angle / 100 + 48)
        date2 = int((target_angle % 100) / 10 + 48)
        date3 = int(target_angle % 10 + 48)
        cmd = bytearray([36, servonum, date1, date2, date3, 35])
        self.pwm_ser.write(cmd)
        # time.sleep(0.05)

    def calculate_checksum(self, data):
        """计算Filter舵机校验码"""
        total_sum = sum(data)
        total_sum = total_sum & 0xFF
        checksum = ~total_sum & 0xFF
        return checksum        

    def calibrate_center_position(self, id):
        """
        舵机中位校准功能（将当前位置自动校正为2048）
        
        参数:
            id:舵机ID (1为旋转轴, 2为升降轴)
        """
        # 构建指令
        header = [0xFF, 0xFF]  # 包头
        data = [
            id,                # ID号
            0x04,             # 数据长度
            0x03,             # 写指令
            0x28,             # 写首地址 (40)
            0x80              # 写入数据 (128，开启中位自动校准)
        ]
        
        # 计算校验码
        checksum = self.calculate_checksum(data)
        data.append(checksum)
        
        # 发送指令
        command = bytes(header + data)
        self.filter_ser.write(command)
        time.sleep(0.02)  # 等待指令执行

    
    def FilterServo(self, id, target_position, target_speed=2400, acceleration=0):
        """
        控制Filter舵机
        
        参数:
            id: 舵机ID (1为旋转轴2048-3630, 2为升降2048-5900)
            target_position: 目标位置 (一圈为0-4095)
            target_speed: 目标速度 (0-3400)
            acceleration: 加速度 (0-255)
        """
        # 将角度转换为Filter舵机的位置值(0-4095)
        target_position = int(target_position)
        
        # 构建指令
        header = [0xFF, 0xFF]
        length = 0x0A
        instruction = 0x03
        address = 0x29
        
        # 拆分位置和速度为高低字节
        position_low = target_position & 0xFF
        position_high = (target_position >> 8) & 0xFF
        speed_low = target_speed & 0xFF
        speed_high = (target_speed >> 8) & 0xFF
        
        # 构建数据包
        data = [
            id,
            length,
            instruction,
            address,
            acceleration,
            position_low,
            position_high,
            0x00,  # 时间低字节
            0x00,  # 时间高字节
            speed_low,
            speed_high
        ]
        
        # 计算校验码
        checksum = self.calculate_checksum(data)
        data.append(checksum)
        
        # 发送指令
        command = bytes(header + data)
        self.filter_ser.write(command)
        # time.sleep(0.02)  # 等待指令执行

    def turn_storage(self,color_id):
        if color_id == 1:          #对应红号色储物盘
            self.PWMServo(9, STORE_1)
            time.sleep(0.02)
        elif color_id == 2:        #对应绿号色储物盘
            self.PWMServo(9, STORE_2)
            time.sleep(0.02)
        elif color_id == 3:         #对应蓝色储物盘
            self.PWMServo(9, STORE_3)
            time.sleep(0.02) 



    def detect_plate(self):
        self.FilterServo(1,ANGLE_CENTER,2700,200)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_3,3400,0)
        time.sleep(0.1)
        self.PWMServo(16,LINE_OPEN)
    def detect_ring_HIGH_from_store(self):
        self.FilterServo(1,ANGLE_CENTER,2700,200)
        time.sleep(0.01)
        self.FilterServo(2,HEIGHT_7,3400,0)
        time.sleep(0.05)
        self.PWMServo(16,LINE_OPEN)

    def detect_ring_LOW_from_store(self):
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.15)
        self.PWMServo(16,HOLD)
        time.sleep(0.2)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.14)
        self.FilterServo(1,ANGLE_CENTER,3000,180)
        time.sleep(0.2)
        self.FilterServo(2,HEIGHT_8,3400,0)

    def detect_ring_LOW_from_ground(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.5)
        self.FilterServo(1,ANLGE_STORE,2700,200)
        time.sleep(0.05)
        self.PWMServo(16,HALF)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.15)
        self.PWMServo(16,HOLD)
        time.sleep(0.2)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.14)
        self.FilterServo(1,ANGLE_CENTER,3000,180)
        time.sleep(0.2)
        self.FilterServo(2,HEIGHT_8,3400,0)
    def detect_ring_LOW_from_HIGH(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.005)
        self.FilterServo(1,ANLGE_STORE,2700,200)
        time.sleep(0.005)
        self.PWMServo(16,HALF)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.15)
        self.PWMServo(16,HOLD)
        time.sleep(0.2)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.14)
        self.FilterServo(1,ANGLE_CENTER,3000,180)
        time.sleep(0.2)
        self.FilterServo(2,HEIGHT_8,3400,0)
        

    def put_from_LOW_to_ground(self):
        self.FilterServo(2,HEIGHT_1,3400,0)
        time.sleep(0.27)
        self.PWMServo(16,LINE_OPEN)


    def detect_plate_ring_LOW(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.01)
        self.FilterServo(1,ANLGE_STORE,2700,200)
        time.sleep(0.01)
        self.PWMServo(16,HALF)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.3)
        self.PWMServo(16,HOLD)
        time.sleep(0.2)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.3)
        self.FilterServo(1,ANGLE_CENTER,3000,200)
        time.sleep(0.35)
        self.FilterServo(2,HEIGHT_3+200,3400,0)

    def from_plate_ring_LOW_to_plate(self):
        self.FilterServo(2,HEIGHT_3,3400,0)
        time.sleep(0.27)
        self.PWMServo(16,LINE_OPEN)
        time.sleep(0.41)
        self.FilterServo(2,HEIGHT_7,3400,0)
        time.sleep(0.01)
    def from_HIGH_to_plate_LOW(self):
        self.FilterServo(2,HEIGHT_3+200,3400,0)
    def from_plate_LOW_to_HIGH(self):
   
        self.FilterServo(2,HEIGHT_7,3400,0)

        
    def put_from_store_to_plate(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.01)
        self.FilterServo(1,ANLGE_STORE,2700,200)
        time.sleep(0.01)
        self.PWMServo(16,HALF)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.2)
        self.PWMServo(16,HOLD)
        time.sleep(0.2)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.2)
        self.FilterServo(1,ANGLE_CENTER,3000,200)
        time.sleep(0.45)
        self.FilterServo(2,HEIGHT_3,3400,0)
        time.sleep(0.7)
        self.PWMServo(16,LINE_OPEN)
        time.sleep(0.42)
        self.FilterServo(2,HEIGHT_7,3400,0)
    def put_from_HIGH_to_plate(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.01)
        self.FilterServo(1,ANLGE_STORE,2700,200)
        self.PWMServo(16,HALF)
        time.sleep(0.6)
        

    def grab_from_ground_to_store(self):
        self.PWMServo(16,HOLD)
        time.sleep(0.47)
        self.FilterServo(2,HEIGHT_4,3400,0)
        time.sleep(0.6)
        self.FilterServo(1,ANLGE_STORE,3400,200)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.1)
        self.PWMServo(16,HALF)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.3)
    
    def from_ring_HIGH_to_ring_LOW(self):
        self.FilterServo(2,HEIGHT_8,3400,0)

    def from_store_to_ground(self):
        self.FilterServo(1,ANGLE_CENTER,3400,200)
        time.sleep(0.3)
        self.PWMServo(16,LINE_OPEN)
        # time.sleep(0.3)
        self.FilterServo(2,HEIGHT_1,3400,0)

    def from_ring_HIGH_to_ground(self):
        self.FilterServo(2,HEIGHT_1,3400,0)
        time.sleep(0.6)

    def  grab_from_plate(self):
        self.cnt1+=1
        self.PWMServo(16,HOLD)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_4,3400,0)
        time.sleep(0.2)
        self.FilterServo(1,ANLGE_STORE,2700,200)
        time.sleep(0.7)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.1)
        self.PWMServo(16,HALF)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.3)
        self.FilterServo(1,ANLGE_STORE-200)
        if(self.cnt1==1 or self.cnt1==2 or self.cnt1==4 or self.cnt1==5):
            self.FilterServo(1,ANGLE_CENTER,273400,200)
            time.sleep(0.3)
            self.PWMServo(16,LINE_OPEN)
            self.FilterServo(2,HEIGHT_3,3400,0)
        
    def hold_maduo_from_detect_HIGH(self):
        self.PWMServo(16,HALF)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.01)
        self.FilterServo(1,ANLGE_STORE,3400,200)
        time.sleep(0.5)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.25)
        self.PWMServo(16,HOLD)
        time.sleep(0.25)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.2)
        self.FilterServo(1,ANGLE_CENTER,3400,200)
        time.sleep(0.35)
        self.FilterServo(2,HEIGHT_2+200,3400,0)
    
    def maduo_down(self):
        time.sleep(0.5)
        self.FilterServo(2,HEIGHT_2,3400,0)
        time.sleep(0.25)
        self.PWMServo(16,LINE_OPEN)
        time.sleep(0.3)
        self.FilterServo(2,HEIGHT_4,3400,0)
    
    def task_over(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.5)
        self.FilterServo(1,ANLGE_STORE+200,2400,200)
        time.sleep(0.5)
        self.FilterServo(2,HEIGHT_3+100,3400,0)
        self.PWMServo(16,HOLD+10)







    def initialize_arm(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.8)
        self.FilterServo(1,ANLGE_STORE,3400,150)
        self.PWMServo(16,HALF)  
        self.PWMServo(16,HALF)  




    def put_ground(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.65)
        self.FilterServo(1,ANLGE_STORE,3000,200)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.3)
        self.PWMServo(16,HOLD)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.3)
        self.FilterServo(1,ANGLE_CENTER,3000,200)
        time.sleep(0.35)
        self.FilterServo(2,HEIGHT_1,3400,0)
        time.sleep(1.1)
        self.PWMServo(16,HALF)

    def put_plate(self):
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.65)
        self.FilterServo(1,ANLGE_STORE,3000,200)
        time.sleep(0.6)
        self.FilterServo(2,HEIGHT_6,3400,0)
        time.sleep(0.3)
        self.PWMServo(16,HOLD)
        self.FilterServo(2,HEIGHT_5,3400,0)
        time.sleep(0.3)
        self.FilterServo(1,ANGLE_CENTER,3000,200)
        time.sleep(0.35)
        self.FilterServo(2,HEIGHT_3,3400,0)
        time.sleep(0.7)
        self.PWMServo(16,HALF)
        

        










