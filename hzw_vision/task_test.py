from Uart import Uart
from Cameracontrol import Cameracontrol
from Servocontrol import Servocontrol
from TaskReceiver import TaskReceiver
import time



#获取二维码
def get_order():
    global color_id
    servocontrol.initialize_arm()
    order = Qr_cam.get_Qr()#字符串
    # order="123+321"
    # print(f"当前任务: {order}")
    #发送二维码到屏幕
    screen_serial.send_order2screen(order)
    #发送二维码到stm32
    stm_serial.send_order(order)
    #获取颜色id
    digits = [int(char) for char in order if char.isdigit()]
    color_id = [digits[i:i + 3] for i in range(0, len(digits), 3)]

#wifi接收任务
def wifi_receive():
    global color_id
    wifi_receiver.start()
    servocontrol.initialize_arm()
    
    try:
        while True:
            order = wifi_receiver.get_latest_task()
            if order:
                digits = [int(char) for char in order if char.isdigit()]
                color_id = [digits[i:i + 3] for i in range(0, len(digits), 3)]
                print(f"当前任务: {order}")
                wifi_receiver.stop()
                break
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        wifi_receiver.stop()

def map_run_test():
    global color_id
    get_order()
    # time.sleep(3)
    servocontrol.detect_plate()
    #一次转盘blog
    while(True):
            current_color_id, coord, _, still_flag= color_cam.get_closer_to_car_center(0.1,2,11500,24000)  
            if(still_flag==1):
                if(stm_serial.taskid==1 and stm_serial.state==0):
                    stm_serial.send_cmd(coord[0],coord[1],0,1,1)
                    break
    servocontrol.initialize_arm()
    time.sleep(5)
    #粗加工blog
    servocontrol.detect_ring_HIGH_from_store()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[0][0],3000,35000,40,70)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,1)
                break
    servocontrol.initialize_arm()
    time.sleep(3)

    #精加工blog
    servocontrol.detect_ring_HIGH_from_store()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[0][0],3000,35000,40,70)
        if(found_flag):
            if(stm_serial.taskid==3 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,3,1)
                break
    servocontrol.initialize_arm()
    time.sleep(3)

    #二次转盘blog
    servocontrol.detect_plate()
    while(True):
        current_color_id, coord, _, still_flag= color_cam.get_closer_to_car_center(0.1,2,11500,24000)  
        if(still_flag==1):
            if(stm_serial.taskid==1 and stm_serial.state==0):
                stm_serial.send_cmd(coord[0],coord[1],0,1,1)
                break

#精调测试，给底盘测试机械中心
def precise_test():
    global color_id
    servocontrol.detect_ring_HIGH_from_store()
    # servocontrol.PWMServo(16,86)
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(1,1800,7000,20,50)
        if(found_flag):
            stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,1,1)#0.
        # #     print(1)
        # if stm_serial.taskid == 1 and stm_serial.state == 1:
        #     servocontrol.put_from_LOW_to_ground()
def detect_precise_test():
    global color_id
    servocontrol.initialize_arm()
    time.sleep
    servocontrol.detect_ring_LOW_from_store()
    # servocontrol.PWMServo(16,86)
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(1,7500,60000,72,110)
        if(found_flag):
            stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,1,1)#
        # #     print(1)
        # if stm_serial.taskid == 1 and stm_serial.state == 1:
        #     servocontrol.put_from_LOW_to_ground()
#粗调测试，给底盘调整blog函数
def rough_test():
    global color_id
    servocontrol.detect_ring_HIGH_from_store()
    while(True):
        #物料
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(1,1800,8000,30,60)
        #转盘
        # ring_coord,found_flag = color_cam.get_precise_color_ring_center(1,7500,60000,72,110)
        if(found_flag):
            stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,1,1)#0.

def plate_ring_test():
    global color_id
    # servocontrol.detect_plate_ring_LOW()
    while(True):
        ring_coord,found_flag,still_flag=color_cam.get_plate_color_ring_center(3,0.15,3,7500,60000,72,110)
        if(still_flag and ring_coord is not None):
            stm_serial.send_cmd(ring_coord[0]/640,ring_coord[1]/480,0,1,1)



def map_test_with_blog():
    global color_id
    get_order()
    # time.sleep(3)
    servocontrol.detect_ring_HIGH_from_store()
    #暂存区blog
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[0][0],1800,8000,20,50)
        # ring_coord,found_flag,still_flag= color_cam.get_color_center(color_id[round-1][0],0.06,1,4000,4500)

        if(found_flag):
            if(stm_serial.taskid==1 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,1,1)
                break
    servocontrol.initialize_arm()
    time.sleep(5)


    #精加工blog
    servocontrol.detect_ring_HIGH_from_store()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[0][0],3000,35000,45,70)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,1)
                break
    servocontrol.initialize_arm()
    time.sleep(4)

    #转盘blog
    servocontrol.detect_ring_HIGH_from_store()
    while(True):
        current_color_id, coord, _, still_flag= color_cam.get_closer_to_car_center(0.1,2,5800,55000)  
        if(still_flag==1):
            if(stm_serial.taskid==3 and stm_serial.state==0):
                stm_serial.send_cmd(coord[0],coord[1],0,3,1)
                break
    servocontrol.initialize_arm()
    time.sleep(4)


    servocontrol.detect_ring_HIGH_from_store()
    #二次暂存区blog
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[0][0],1800,8000,35,50)
        # ring_coord,found_flag,still_flag= color_cam.get_color_center(color_id[round-1][0],0.06,1,4000,4500)

        if(found_flag):
            if(stm_serial.taskid==1 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,1,1)
                break
    servocontrol.initialize_arm()
    time.sleep(5)

    #二次精加工blog
    servocontrol.detect_ring_HIGH_from_store()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[0][0],3000,35000,40,70)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,1)
                break
    servocontrol.initialize_arm()
    time.sleep(5)


    #二次转盘blog
    servocontrol.detect_ring_HIGH_from_store()
    while(True):
        current_color_id, coord, _, still_flag= color_cam.get_closer_to_car_center(0.1,2,5800,55000)  
        if(still_flag==1):
            if(stm_serial.taskid==3 and stm_serial.state==0):
                stm_serial.send_cmd(coord[0],coord[1],0,3,1)
                break
    servocontrol.initialize_arm()

def plate_test():
    color_id = [[1,2,3],[3,2,1]]
    servocontrol.turn_storage(1)
    servocontrol.detect_ring_HIGH_from_store()

#第一次粗调
    first_color = None
    while(True):
        #*********************************需要改转盘颜色面积*****************************
        current_color_id, coord, _, stillflag= color_cam.get_closer_to_car_center(0.1,2,5800,55000)
        if(stillflag==1): 
            if(stm_serial.taskid==3 and stm_serial.state==0):
                stm_serial.send_cmd(coord[0],coord[1],0,3,1)
                first_color = current_color_id
                break
    
    
#精调色环
    servocontrol.from_HIGH_to_plate_LOW()
    second_color = None
    flag=0
    while(True):
        #*********************************需要改转盘颜色面积和环半径*****************************

        color, ring_coord, _, still_flag= color_cam.get_closer_to_car_color_ring(0.1,2,5500,55000,35,56)
        if(still_flag==1):
            flag=1
        if(color!=first_color and ring_coord is not None and flag==1):
            if(stm_serial.taskid==3 and stm_serial.state==2):
                stm_serial.send_cmd(ring_coord[0]/640,ring_coord[1]/480,0,3,3)
                second_color = color
        elif(stm_serial.taskid==3 and stm_serial.state==4):
            break

#放第一个物料
    servocontrol.from_plate_LOW_to_HIGH()
    cnt=None
    flag=0
    num=1
    while(True):#************************需要改色块面积************************
        now_color,coord,found_flag,still_flag = color_cam.get_closer_to_car_center(0.2,2,11500,69000)
        if(now_color==second_color and flag==0):
            cnt=0
        elif(now_color!=second_color and flag==0):
            cnt=1
            flag=1
        if(cnt==1):#放第一个
            if(now_color==1 and still_flag==1 and num==1):
                servocontrol.put_from_store_to_plate()
                num+=1
                #放第二个
            elif(now_color==2 and still_flag==1 and num==2):
                servocontrol.put_from_store_to_plate()
                num+=1
                #放第三个
            elif(now_color==3 and still_flag==1 and num==3):
                servocontrol.put_from_store_to_plate()
                stm_serial.send_cmd(0,0,0,3,5)
                break


    servocontrol.initialize_arm()
    time.sleep(0.1)
    
def plate_baodi_test():
    servocontrol.detect_ring_HIGH_from_store() 
    #粗调
    first_color=None
    flag=0
    cnt=0
    while(True):
        #*********************************需要改转盘颜色面积*****************************
        current_color_id, coord, _, stillflag= color_cam.get_closer_to_car_center(0.1,2,5800,55000)
        if(stillflag==1 and first_color==None): 
            first_color=current_color_id

        if(current_color_id==first_color and flag==0):
            cnt=0
        elif(current_color_id!=first_color and flag==0):
            cnt=1
            flag=1
        if(cnt==1):#红色
            if(stillflag==1 and current_color_id==1): 
                servocontrol.detect_plate_ring_LOW()
                stm_serial.send_cmd(coord[0],coord[1],0,3,1)
                break

    stm_serial.wait_for_signal(3,2)
    #blog到位放第一个物料
    servocontrol.from_plate_ring_LOW_to_plate()
    num=2
    while(True):#*********************************需要改转盘颜色面积*****************************
        now_color,coord,found_flag,still_flag = color_cam.get_closer_to_car_center(0.1,2,5800,55000)
        #放第二个物料
        if(now_color==2 and still_flag==1 and num==2):
            servocontrol.put_from_store_to_plate()
            num=3
        #放第三个物料
        elif(now_color==3 and still_flag==1 and num==3):
            servocontrol.put_from_store_to_plate()
            stm_serial.send_cmd(0,0,0,3,3)
            break
    servocontrol.initialize_arm()
    time.sleep(0.1)

#***********************初始化***********************
#初始化通信与机械臂
order = None
#初始化stm32串口
stm_serial = Uart('/dev/ttyAMA0',115200)
#初始化屏幕串口
screen_serial = Uart('/dev/ttyAMA2',9600)
#初始化机械臂
servocontrol = Servocontrol()
#初始化二维码摄像头
Qr_cam = Cameracontrol(1)
#初始化色块摄像头
color_cam = Cameracontrol(0)
#开启stm32接收线程
stm_serial.start_receive_stm32()
global color_id
#初始化wifi接收
# wifi_receiver = TaskReceiver()



#************************test**********

#跑图
# map_run_test()
#精调测试
# precise_test()
#粗调测试
# rough_test()
# plate_ring_test()


#***********决赛test*****************
# map_test_with_blog()
# plate_test()
# plate_baodi_test()
# rough_test()
precise_test()