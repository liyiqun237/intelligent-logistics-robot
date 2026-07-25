from Uart import Uart
from Cameracontrol import Cameracontrol
from Servocontrol import Servocontrol
from TaskReceiver import TaskReceiver
import time

def calculate_next_target(task_code, current_position, sorted_colors, material_spacing=15):
    """
    计算下一个目标物料的位置和移动距离
    
    参数:
    task_code (list): 任务码列表，如[1,2,3]表示抓取顺序为红绿蓝
    current_position (int): 当前位置索引，0表示第一个物料位置，1表示第二个，2表示第三个
    sorted_colors (list): 物料排序列表（从右到左）
    material_spacing (int): 物料之间的间距，单位为mm，默认150mm
    
    返回:
    tuple: (distance, direction)，其中distance为移动距离(mm)，direction为移动方向(1为左，-1为右)
    next_color (int): 下一个目标颜色的ID，如果没有下一个目标则返回None
    """
    # 检查当前位置是否有效
    if current_position < 0 or current_position >= len(task_code):
        return 0, 0, None
    
    # 如果已经是最后一个物料，返回None
    if current_position + 1 >= len(task_code):
        return 0, 0, None
    
    # 获取当前颜色和下一个颜色
    current_color = task_code[current_position]
    next_color = task_code[current_position + 1]
    
    # 找到当前颜色和下一个颜色在排序中的位置
    try:
        current_index = sorted_colors.index(current_color)
        next_index = sorted_colors.index(next_color)
    except ValueError:
        return 0, 0, None
    
    # 计算移动方向和距离
    if next_index > current_index:  # 向左移动（因为是从右到左排序）
        direction = 1  # 向左
        distance = (next_index - current_index) * material_spacing
    else:  # 向右移动
        direction = -1  # 向右
        distance = (current_index - next_index) * material_spacing
    
    return distance, direction, next_color

#获取二维码
def get_order():
    global color_id
    servocontrol.initialize_arm()
    servocontrol.turn_storage(1)
    order = Qr_cam.get_Qr()#字符串
    print(f"当前任务: {order}")
    # order="123+321"
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
#移动到转盘    
def move_to_turn_plate(round):
    global color_id
    time.sleep(0.5)
    if(round==2):
        time.sleep(3)
    #展开机械臂
    servocontrol.detect_plate()
    servocontrol.turn_storage(color_id[round-1][0])
    send_flag=0
    first_flag=0
    for target_color_id in color_id[round-1]:
        first_flag+=1
        while(True):
            current_color_id, coord, _, still_flag= color_cam.get_closer_to_car_center(0.1,2,11500,24000)  
            if(still_flag==1):
                if(stm_serial.taskid==1 and stm_serial.state==0):
                    if(send_flag==0):
                        stm_serial.send_cmd(coord[0],coord[1],0,1,1)
                        send_flag=1

                elif(stm_serial.taskid==1 and stm_serial.state==2):#此时底盘已blog
                    if(target_color_id==current_color_id):
                        if(first_flag!=1):
                            servocontrol.turn_storage(target_color_id)
                        time.sleep(0.01)
                        servocontrol.grab_from_plate()
                        color_cam.close_windows()
                        break
    
    #抓完三个
    stm_serial.send_cmd(0,0,0,1,3)
    #提前转到第一个
    servocontrol.turn_storage(color_id[round-1][0])
    color_cam.close_windows()


#移动到粗加工
def move_to_rough(round):
    global color_id
    time.sleep(7)
    servocontrol.detect_ring_HIGH_from_store()
#第一次粗定位
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][0],3000,35000,40,70)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,1)
                break
#精调第一个环  
    time.sleep(0.4) 
    servocontrol.detect_ring_LOW_from_HIGH()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][0],7500,60000,72,110)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==2):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,3)
            
            elif(stm_serial.taskid==2 and stm_serial.state==4):
                servocontrol.put_from_LOW_to_ground()
                servocontrol.turn_storage(color_id[round-1][1])
                time.sleep(0.5)
                stm_serial.send_cmd(0,0,0,2,5)
                break
        
#精调第二个环
    servocontrol.detect_ring_LOW_from_ground()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][1],7500,60000,72,110)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==6):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,7)

            elif(stm_serial.taskid==2 and stm_serial.state==8):
                servocontrol.put_from_LOW_to_ground()
                servocontrol.turn_storage(color_id[round-1][2])
                time.sleep(0.5)
                stm_serial.send_cmd(0,0,0,2,9)
                break
#精调第三个环
    servocontrol.detect_ring_LOW_from_ground()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][2],7500,60000,72,110)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==10):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,11)

            elif(stm_serial.taskid==2 and stm_serial.state==12):
                servocontrol.put_from_LOW_to_ground()
                servocontrol.turn_storage(color_id[round-1][0])
                time.sleep(0.5)
                stm_serial.send_cmd(0,0,0,2,13)
                break
#抓回第一个物料
    stm_serial.wait_for_signal(2, 14)
    servocontrol.grab_from_ground_to_store()
    stm_serial.send_cmd(0,0,0,2,15)
    servocontrol.turn_storage(color_id[round-1][1])
    servocontrol.from_store_to_ground()
#抓回第二个物料
    stm_serial.wait_for_signal(2, 16)
    servocontrol.grab_from_ground_to_store()
    stm_serial.send_cmd(0,0,0,2,17)
    servocontrol.turn_storage(color_id[round-1][2])
    servocontrol.from_store_to_ground()
#抓回第三个物料
    stm_serial.wait_for_signal(2, 18)
    servocontrol.grab_from_ground_to_store()
    stm_serial.send_cmd(0,0,0,2,19)
    servocontrol.FilterServo(1,3300,2700,200   )
    servocontrol.turn_storage(color_id[round-1][0])

def move_to_fine(round):
    global color_id
    time.sleep(6)
    servocontrol.detect_ring_HIGH_from_store()
#第一次粗定位
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][0],4000,45000,45,70)
        if(found_flag):
            if(stm_serial.taskid==3 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,3,1)
                break

    if(round==1):#放底层
    #精调第一个环   
        servocontrol.detect_ring_LOW_from_HIGH()
        while(True):
            ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][0],7500,60000,72,110)
            if(found_flag):
                if(stm_serial.taskid==3 and stm_serial.state==2):
                    stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,3,3)
                
                elif(stm_serial.taskid==3 and stm_serial.state==4):
                    servocontrol.put_from_LOW_to_ground()
                    servocontrol.turn_storage(color_id[round-1][1])
                    time.sleep(0.5)
                    stm_serial.send_cmd(0,0,0,3,5)
                    break
            
    #精调第二个环
        servocontrol.detect_ring_LOW_from_ground()
        while(True):
            ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][1],7500,60000,72,110)
            if(found_flag):
                if(stm_serial.taskid==3 and stm_serial.state==6):
                    stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,3,7)

                elif(stm_serial.taskid==3 and stm_serial.state==8):
                    servocontrol.put_from_LOW_to_ground()
                    servocontrol.turn_storage(color_id[round-1][2])
                    time.sleep(0.5)
                    stm_serial.send_cmd(0,0,0,3,9)
                    break
    #精调第三个环
        servocontrol.detect_ring_LOW_from_ground()
        while(True):
            ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][2],7500,60000,72,110)
            if(found_flag):
                if(stm_serial.taskid==3 and stm_serial.state==10):
                    stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,3,11)

                elif(stm_serial.taskid==3 and stm_serial.state==12):
                    servocontrol.put_from_LOW_to_ground()
                    time.sleep(0.5)
                    stm_serial.send_cmd(0,0,0,3,13)
                    servocontrol.initialize_arm()
                    break
        
    elif(round==2):#码垛
        #第一次码垛
        servocontrol.hold_maduo_from_detect_HIGH()
        stm_serial.wait_for_signal(3,2)
        servocontrol.maduo_down()
        stm_serial.send_cmd(0,0,0,3,3)
        servocontrol.turn_storage(color_id[round-1][1])
        time.sleep(0.7)

        #第二次码垛
        servocontrol.hold_maduo_from_detect_HIGH()
        stm_serial.wait_for_signal(3,4)
        servocontrol.maduo_down()
        stm_serial.send_cmd(0,0,0,3,5)
        servocontrol.turn_storage(color_id[round-1][2])
        time.sleep(0.7)


        #第三次码垛
        servocontrol.hold_maduo_from_detect_HIGH()
        stm_serial.wait_for_signal(3,6)
        servocontrol.maduo_down()
        stm_serial.send_cmd(0,0,0,3,7)

def task_over():
    global color_id
    servocontrol.task_over()


def move_to_zancun(round):
    global color_id
    servocontrol.turn_storage(color_id[round-1][0])
    if(round==1):
        time.sleep(7)
    elif(round==2):
        time.sleep(4)
    servocontrol.detect_ring_HIGH_from_store()
    stm_serial.wait_for_signal(1,0)
    while(True):
        sorted_colors, found_flag, still_flag = color_cam.get_color_blocks_sorted_by_x(0.5, 2, 1800, 6300)
        if found_flag and still_flag:
            print(f"{sorted_colors}")
            break
    
    #精到第一个物料
    while(True):
        #*********************************需要改*****************************
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][0],1800,7000,20,50)

        #ring_coord,found_flag,still_flag= color_cam.get_color_center(color_id[round-1][0],0.06,1,4000,4500)

        if(found_flag):
            if(stm_serial.taskid==1 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,1,1)
            elif(stm_serial.taskid==1 and stm_serial.state==2):
                break
    time.sleep(0.2) 
    #抓取第一个物料
    servocontrol.from_ring_HIGH_to_ground()
    servocontrol.grab_from_ground_to_store()
    distance1, direction1, _ = calculate_next_target(color_id[round-1], 0, sorted_colors)

    stm_serial.send_cmd(0,0,0,distance1,direction1+1)
    servocontrol.turn_storage(color_id[round-1][1])
    servocontrol.from_store_to_ground()

    #抓取第二个物料
    stm_serial.wait_for_signal(1,4)
    servocontrol.grab_from_ground_to_store()
    distance2, direction2, _ = calculate_next_target(color_id[round-1], 1, sorted_colors)
    stm_serial.send_cmd(0,0,0,distance2+1,direction2+1)
    servocontrol.turn_storage(color_id[round-1][2])
    servocontrol.from_store_to_ground()

    #抓取第三个物料
    stm_serial.wait_for_signal(1,6)
    servocontrol.grab_from_ground_to_store()
    third_color = color_id[round-1][2]
    third_color_index = sorted_colors.index(third_color)
    
    stm_serial.send_cmd(0,0,0,1,third_color_index)
    servocontrol.FilterServo(1,3300,2700,200 )
    servocontrol.turn_storage(color_id[round-1][0])
def movet_to_zancun_baodi(round):
    global color_id

def move_to_jingjiagong(round):
    global color_id
    time.sleep(6)
    servocontrol.detect_ring_HIGH_from_store()
#第一次粗定位
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][0],3000,35000,40,70)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==0):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,1)
                break
#精调第一个环  
    time.sleep(0.4) 
    servocontrol.detect_ring_LOW_from_HIGH()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][0],7500,60000,72,110)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==2):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,3)
            
            elif(stm_serial.taskid==2 and stm_serial.state==4):
                servocontrol.put_from_LOW_to_ground()
                servocontrol.turn_storage(color_id[round-1][1])
                time.sleep(0.5)
                stm_serial.send_cmd(0,0,0,2,5)
                break
        
#精调第二个环
    servocontrol.detect_ring_LOW_from_ground()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][1],7500,60000,72,110)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==6):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,7)

            elif(stm_serial.taskid==2 and stm_serial.state==8):
                servocontrol.put_from_LOW_to_ground()
                servocontrol.turn_storage(color_id[round-1][2])
                time.sleep(0.5)
                stm_serial.send_cmd(0,0,0,2,9)
                break
#精调第三个环
    servocontrol.detect_ring_LOW_from_ground()
    while(True):
        ring_coord,found_flag = color_cam.get_precise_color_ring_center(color_id[round-1][2],7500,60000,72,110)
        if(found_flag):
            if(stm_serial.taskid==2 and stm_serial.state==10):
                stm_serial.send_cmd(ring_coord[0],ring_coord[1],0,2,11)

            elif(stm_serial.taskid==2 and stm_serial.state==12):
                servocontrol.put_from_LOW_to_ground()
                servocontrol.turn_storage(color_id[round-1][0])
                time.sleep(0.5)
                stm_serial.send_cmd(0,0,0,2,13)
                break
#抓回第一个物料
    stm_serial.wait_for_signal(2, 14)
    servocontrol.grab_from_ground_to_store()
    stm_serial.send_cmd(0,0,0,2,15)
    servocontrol.turn_storage(color_id[round-1][1])
    servocontrol.from_store_to_ground()
#抓回第二个物料
    stm_serial.wait_for_signal(2, 16)
    servocontrol.grab_from_ground_to_store()
    stm_serial.send_cmd(0,0,0,2,17)
    servocontrol.turn_storage(color_id[round-1][2])
    servocontrol.from_store_to_ground()
#抓回第三个物料
    stm_serial.wait_for_signal(2, 18)
    servocontrol.grab_from_ground_to_store()
    stm_serial.send_cmd(0,0,0,2,19)
    servocontrol.FilterServo(1,3300,2700,200   )
    servocontrol.turn_storage(color_id[round-1][0])
    
def move_to_plate(round):
    global color_id
    time.sleep(4)
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
        color, ring_coord, _, still_flag= color_cam.get_closer_to_car_color_ring(0.1,2,5800,55000,30,56)
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
        now_color,coord,found_flag,still_flag = color_cam.get_closer_to_car_center(0.1,2,5800,55000)
        if(now_color==second_color and flag==0):
            cnt=0
        elif(now_color!=second_color and flag==0):
            cnt=1
            flag=1
        if(cnt==1):#放第一个
            if(now_color==color_id[round-1][0] and still_flag==1 and num==1):
                servocontrol.put_from_store_to_plate()
                num+=1
                #放第二个
            elif(now_color==color_id[round-1][1] and still_flag==1 and num==2):
                servocontrol.put_from_store_to_plate()
                num+=1
                #放第三个
            elif(now_color==color_id[round-1][2] and still_flag==1 and num==3):
                servocontrol.put_from_store_to_plate()
                stm_serial.send_cmd(0,0,0,3,5)
                break


    servocontrol.initialize_arm()
    time.sleep(0.1)


def move_to_plate_baodi(round):
    global color_id
    time.sleep(4)
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
        if(cnt==1):
            if(stillflag==1 and current_color_id==color_id[round-1][0]): 
                servocontrol.detect_plate_ring_LOW()
                stm_serial.send_cmd(coord[0],coord[1],0,3,1)
                break

    stm_serial.wait_for_signal(3,2)
    #blog到位放第一个物料
    servocontrol.from_plate_ring_LOW_to_plate()
    num=2
    while(True):#*********************************需要改转盘颜色面积*****************************
        now_color,coord,found_flag,still_flag = color_cam.get_closer_to_car_center(0.1,2,5800,5500)
        #放第二个物料
        if(now_color==color_id[round-1][1] and still_flag==1 and num==2):
            servocontrol.put_from_store_to_plate()
            num=3
        #放第三个物料
        elif(now_color==color_id[round-1][2] and still_flag==1 and num==3):
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



#***********************初赛***********************

# get_order()
# move_to_turn_plate(1)
# move_to_rough(1)
# move_to_fine(1)
# move_to_turn_plate(2)
# move_to_rough(2)
# move_to_fine(2)
# task_over()

#***********************决赛***********************
#需要测量转盘抓取高度的色环面积和半径
get_order()
move_to_zancun(1)
move_to_jingjiagong(1)
move_to_plate_baodi(1)
move_to_zancun(2)
move_to_jingjiagong(2)
move_to_plate_baodi(2)
task_over()
