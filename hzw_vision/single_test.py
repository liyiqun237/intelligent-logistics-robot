import serial
import struct
import cv2
import numpy as np
import serial
import time
from collections import Counter
import threading
from Cameracontrol import Cameracontrol
from Servocontrol import Servocontrol
from Uart import Uart
#爪子中心：(0.492,0.42)
taskid = None
state = None
order = None
color_id =None
#实例化
stm_serial = Uart('/dev/ttyAMA2',115200)
screen_serial = Uart('/dev/ttyUSB0',9600)
servocontrol = Servocontrol()
Qr_cam = Cameracontrol(1)
color_cam = Cameracontrol(0)


#摄像头测试
# while(True):
#     current_color_id, coord, _, still_flag= color_cam.get_closest_center((0.49,0.42)) 
#     if(still_flag):
#         # if(coord[0]<0.3):
#             # color_cam.close_windows()
#         #     color_cam.close_cam()

#             # time.sleep(10)
#             print("over")
#             break
# color_cam.close_windows()

# stm_serial.start_update(4,1,color_cam)
# time.sleep(5)   
# stm_serial.stop_update()

# # color_cam =Cameracontrol(0)
# color_cam.close_windows()
# stm_serial.start_update(4,1,color_cam)
# time.sleep(5)
# stm_serial.stop_update()
# color_cam.close_windows()
# while(True):
#     current_color_id, coord, _, still_flag= color_cam.get_closest_center((0.49,0.42)) 
#     if(still_flag):
#         # if(coord[0]<0.3):
#             # color_cam.close_windows()
#         #     color_cam.close_cam()

#             # time.sleep(10)
#             print("over")
#             break
# color_cam.close_windows()
    
# stm_serial.start_update(4,1,color_cam)
# time.sleep(5)
# stm_serial.stop_update()
# stm_serial.start_update(4,1,color_cam)
# time.sleep(10)
# stm_serial.stop_update()





#------------------------------开始---------------------------------
stm_serial.send_cmd(0,0,0,1,0)
print("wait for stm32 begin")
stm_serial.wait_for_signal(0, 1, success_message="begin!!!")
#--------------------------------扫码转发---------------------------------
#获取二维码
order = Qr_cam.get_Qr()   #没识别到二维码会一直阻塞      
digits = [int(char) for char in order if char.isdigit()]
color_id = [digits[i:i + 3] for i in range(0, len(digits), 3)] #得到一个颜色序号的二维数组
#发送二维码到屏幕
screen_serial.send_order2screen(order)
#发送二维码到stm32
stm_serial.send_order(order)
#-------------------------------第一次任务---------------------------------
#--------------------------------转盘抓取---------------------------------
#粗到转盘
stm_serial.wait_for_signal(1, 1, success_message="reached_turn_plate")
#伸出机械臂
time.sleep(0.5)
servocontrol.extend_arm_2()
#依次抓取物块
send_flag=0
for target_color_id in color_id[0]:
    while(True):
        current_color_id, coord, _, still_flag= color_cam.get_closest_center((0.49,0.42))  
        if(still_flag==1 ):#一切基于静止时进行
            if(send_flag==0):#只进一次
                stm_serial.send_cmd(coord[0],coord[1],0,2,0)
                send_flag =1#表示已经发过坐标给stm32了
                stm_serial.wait_for_signal(2, 1, success_message="reached")
                time.sleep(0.8)
            #抓取
            if(target_color_id ==current_color_id):
                servocontrol.grab_1(target_color_id)
                break
color_cam.close_windows()
# color_cam.close_cam()
stm_serial.send_cmd(0,0,0,3,1) #抓取结束
servocontrol.turn_storage(color_id[0][0])
servocontrol.fold_arm()

#---------------------------------第一次粗加工---------------------------------
#粗到粗加工区第一个圆环
stm_serial.wait_for_signal(3, 1, success_message="reached_ring_1")
#伸出机械臂
servocontrol.detect_ring()
#开启精调线程
stm_serial.start_update(4,color_id[0][0],color_cam)
servocontrol.turn_storage(color_id[0][0])
#精到粗加工区第一个圆环
stm_serial.wait_for_signal(4, 0,timeout=9, success_message="reached_2_ring_1")
stm_serial.stop_update()
color_cam.close_windows()
#放下第一个物块
servocontrol.put_1(color_id[0][0])
stm_serial.send_cmd(0,0,0,4,1)

#粗到粗加工区第二个圆环
stm_serial.wait_for_signal(3, 1, success_message="reached_ring_2")
#开启精调线程
stm_serial.start_update(4,color_id[0][1],color_cam)
servocontrol.turn_storage(color_id[0][1])
#精到粗加工区第二个圆环
stm_serial.wait_for_signal(4, 0,timeout=9,  success_message="reached_2_ring_2")
stm_serial.stop_update()
color_cam.close_windows()
#放下第二个物块
servocontrol.put_1(color_id[0][1])
stm_serial.send_cmd(0,0,0,4,1)

#粗到第三个圆环
stm_serial.wait_for_signal(3, 1, success_message="reached_ring_3")
#开启精调线程
stm_serial.start_update(4,color_id[0][2],color_cam)
servocontrol.turn_storage(color_id[0][2])
#精到第三个圆环
stm_serial.wait_for_signal(4, 0, timeout=9,  success_message="reached_2_ring_3")
stm_serial.stop_update()
color_cam.close_windows()
#放下第三个物块
servocontrol.put_1(color_id[0][2])
stm_serial.send_cmd(0,0,0,4,1)
servocontrol.turn_storage(color_id[0][0])

#回到第一个圆环
stm_serial.wait_for_signal(3, 1, success_message="reached_3_ring_1")
#抓回第一个物块
servocontrol.grab_2(color_id[0][0])
stm_serial.send_cmd(0,0,0,4,2)
servocontrol.turn_storage(color_id[0][1])

#回到第二个圆环
stm_serial.wait_for_signal(3, 1, success_message="reached_3_ring_2")
#抓回第二个物块
servocontrol.grab_2(color_id[0][1])
stm_serial.send_cmd(0,0,0,4,3)
servocontrol.turn_storage(color_id[0][2])

#回到第三个圆环
stm_serial.wait_for_signal(3, 1, success_message="reached_3_ring_3")
#抓回第三个物块
servocontrol.grab_2(color_id[0][2])
stm_serial.send_cmd(0,0,0,5,0)
print("第一次粗加工完成")
servocontrol.turn_storage(color_id[0][0])
servocontrol.fold_arm()

# ---------------------------------第一次精加工---------------------------------
#粗到精加工第一个圆环
stm_serial.wait_for_signal(5, 1, success_message="reached_fine_ring_1")
#伸出机械臂
servocontrol.detect_ring()
#开启精调线程
stm_serial.start_update(6,color_id[0][0],color_cam)
servocontrol.turn_storage(color_id[0][0])
#精到第一个圆环
stm_serial.wait_for_signal(6, 0, timeout=9, success_message="reached_2_fine_ring_1")
stm_serial.stop_update()
color_cam.close_windows()
#放下第一个物块
servocontrol.put_1(color_id[0][0])
stm_serial.send_cmd(0,0,0,6,1)

#粗到精加工第二个圆环
stm_serial.wait_for_signal(5, 1, success_message="reached_fine_ring_2")
#开启精调线程
stm_serial.start_update(6,color_id[0][1],color_cam)
servocontrol.turn_storage(color_id[0][1])
#精到第二个圆环
stm_serial.wait_for_signal(6, 0, timeout=9, success_message="reached_2_fine_ring_2")
stm_serial.stop_update()
color_cam.close_windows()
#放下第二个物块
servocontrol.put_1(color_id[0][1])
stm_serial.send_cmd(0,0,0,6,1)

#粗到精加工第三个圆环
stm_serial.wait_for_signal(5, 1, success_message="reached_fine_ring_3")
#开启精调线程
stm_serial.start_update(6,color_id[0][2],color_cam)
servocontrol.turn_storage(color_id[0][2])
#精到第三个圆环
stm_serial.wait_for_signal(6, 0, timeout=8, success_message="reached_2_fine_ring_3")
stm_serial.stop_update()
color_cam.close_windows()
#放下第三个物块
servocontrol.put_1(color_id[0][2])
stm_serial.send_cmd(0,0,0,7,0)
servocontrol.fold_arm()
print("第一轮任务完成")
servocontrol.turn_storage(color_id[1][0])
#--------------------------------第二轮任务---------------------------------
#--------------------------------转盘抓取---------------------------------
stm_serial.wait_for_signal(7, 1, success_message="reached_turn_plate_2")
#伸出机械臂
time.sleep(1)
servocontrol.extend_arm_2()
#依次抓取物块
send_flag = 0  # 重置标志位
for target_color_id in color_id[1]:  # 使用第二组颜色顺序
    while(True):
        current_color_id, coord, _, still_flag= color_cam.get_closest_center((0.49,0.42),duration=0.2,still_threshold=3)  
        if(still_flag==1 ):
            if(send_flag==0):
                stm_serial.send_cmd(coord[0],coord[1],0,8,0)  # 精调改为8,0
                send_flag =1
                stm_serial.wait_for_signal(8, 1, success_message="reached_2")
                time.sleep(1) 
            if(target_color_id ==current_color_id):
                servocontrol.grab_1(target_color_id)
                break
color_cam.close_windows()
stm_serial.send_cmd(0,0,0,9,1) #抓取结束
servocontrol.turn_storage(color_id[1][0])
servocontrol.fold_arm()

#--------------------------------第二轮粗加工---------------------------------
#粗到第一个圆环
stm_serial.wait_for_signal(9, 1, success_message="reached_ring_1_second")
#伸出机械臂
servocontrol.detect_ring()
#开启精调线程
stm_serial.start_update(10,color_id[1][0],color_cam)
servocontrol.turn_storage(color_id[1][0])
#精到第一个圆环
stm_serial.wait_for_signal(10, 0, timeout=9,success_message="reached_2_ring_1_second")
color_cam.close_windows()
#放下第一个物块
servocontrol.put_1(color_id[1][0])
stm_serial.send_cmd(0,0,0,10,1)
#粗到第二个圆环
stm_serial.wait_for_signal(9, 1, success_message="reached_ring_2_second")
#开启精调线程
stm_serial.start_update(10,color_id[1][1],color_cam)
servocontrol.turn_storage(color_id[1][1])
#精到第二个圆环
stm_serial.wait_for_signal(10, 0, timeout=9,success_message="reached_2_ring_2_second")
stm_serial.stop_update()
color_cam.close_windows()
#放下第二个物块
servocontrol.put_1(color_id[1][1])
stm_serial.send_cmd(0,0,0,10,1)

#粗到第三个圆环
stm_serial.wait_for_signal(9, 1, success_message="reached_ring_3_second")
#开启精调线程
stm_serial.start_update(10,color_id[1][2],color_cam)
servocontrol.turn_storage(color_id[1][2])
#精到第三个圆环
stm_serial.wait_for_signal(10, 0, timeout=9,success_message="reached_2_ring_3_second")
stm_serial.stop_update()
color_cam.close_windows()
#放下第三个物块
servocontrol.put_1(color_id[1][2])
servocontrol.turn_storage(color_id[1][0])
stm_serial.send_cmd(0,0,0,10,1)

# 第二轮粗加工回收
#回到第一个圆环
stm_serial.wait_for_signal(9, 1, success_message="reached_3_ring_1_second")
#抓回第一个物块
servocontrol.grab_2(color_id[1][0])
servocontrol.turn_storage(color_id[1][1])
stm_serial.send_cmd(0,0,0,10,2)

#回到第二个圆环
stm_serial.wait_for_signal(9, 1, success_message="reached_3_ring_2_second")
#抓回第二个物块
servocontrol.grab_2(color_id[1][1])
servocontrol.turn_storage(color_id[1][2])
stm_serial.send_cmd(0,0,0,10,3)

#回到第三个圆环
stm_serial.wait_for_signal(9, 1, success_message="reached_3_ring_3_second")
#抓回第三个物块
servocontrol.grab_2(color_id[1][2])
stm_serial.send_cmd(0,0,0,11,0)
print("第二轮粗加工完成")
servocontrol.turn_storage(color_id[1][0])
servocontrol.fold_arm()
#--------------------------------第二轮精加工---------------------------------
#粗到精加工第一个圆环
stm_serial.wait_for_signal(11, 1, success_message="reached_fine_ring_1_2")
#伸出机械臂
servocontrol.extend_arm()
#开启精调线程
stm_serial.start_update(12,color_id[1][0],color_cam)
servocontrol.turn_storage(color_id[1][0])
#精到第一个圆环
stm_serial.wait_for_signal(12, 0, timeout=9,success_message="reached_2_fine_ring_1_2")
stm_serial.stop_update()
color_cam.close_windows()
#放下第一个物块
servocontrol.put_2(color_id[1][0])
stm_serial.send_cmd(0,0,0,12,1)

#粗到精加工第二个圆环
stm_serial.wait_for_signal(11, 1, success_message="reached_fine_ring_2_2")
#伸出机械臂
servocontrol.extend_arm()
#开启精调线程
stm_serial.start_update(12,color_id[1][1],color_cam)
servocontrol.turn_storage(color_id[1][1])
#精到第二个圆环
stm_serial.wait_for_signal(12, 0,timeout=9, success_message="reached_2_fine_ring_2_2")
stm_serial.stop_update()
color_cam.close_windows()
#放下第二个物块
servocontrol.put_2(color_id[1][1])
stm_serial.send_cmd(0,0,0,12,1)

#粗到精加工第三个圆环
stm_serial.wait_for_signal(11, 1, success_message="reached_fine_ring_3_2")
#伸出机械臂
servocontrol.extend_arm()
#开启精调线程
stm_serial.start_update(12,color_id[1][2],color_cam)
servocontrol.turn_storage(color_id[1][2])
#精到第三个圆环
stm_serial.wait_for_signal(12, 0,timeout=9, success_message="reached_2_fine_ring_3_2")
stm_serial.stop_update()
color_cam.close_windows()
#放下第三个物块
servocontrol.put_2(color_id[1][2])
stm_serial.send_cmd(0,0,0,13,0)
print("第二轮任务完成")
stm_serial.wait_for_signal(13, 1, success_message="end!!!")
















































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































