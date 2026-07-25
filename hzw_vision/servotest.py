import serial
import time
from Servocontrol import Servocontrol
from Uart import Uart
# screen_serial = Uart('/dev/ttyUSB0',9600)
# order = "213+121"
# screen_serial.send_order2screen(order)
servo = Servocontrol()
# servo.calibrate_center_position(2)
#复位到低处
# servo.detect_ring_LOW_from_store()
#servo.initialize_arm()
# servo.PWMServo(16,86)
servo.PWMServo(16,29)
# servo.turn_storage(3)
#地面抓取物料高度
# servo.FilterServo(2,2048,3400,0)

#抓取物料后抬升高度
servo.FilterServo(2,3860,3400,0)


#旋转轴位置确定
# servo.FilterServo(1,2048,2400,100)

#抓取储物区物料高度
# servo.FilterServo(2,3950,3400,0)

#储存物料后抬升高度
# servo.FilterServo(2,5300,3400,0)


#从地面抓到储物盘
# servo.grab_from_ground_to_store()
#*************************************************************************************
#从储物盘抓到地面精调高度
# servo.detect_ring_LOW_from_store()
# servo.detect_ring_LOW_from_ground()
# servo.put_from_LOW_to_ground()
#*************************************************************************************

#从储物盘取物料到转盘
# servo.FilterServo(2,3940,3400,0)









#以上为决赛
#*************************************************************************************








# servo.FilterServo(2,2180,3400,0)
# time.sleep(0.4)
# servo.PWMServo(9, 0)
# servo.FilterServo(2,2500,3400,200) 

# time.sleep(0.1)

# servo.turn_storage(3)
# servo.FilterServo(2,5650,3400,0)
# servo.initialize_arm()
# servo.detect_ring_LOW_from_store()
# servo.detect_ring_LOW_from_ground()
# servo.detect_plate()
# servo.detect_ring_HIGH_from_store()
# servo.detect_ring_LOW_from_HIGH()
# servo.put_from_LOW_to_ground()
# servo.grab_from_ground_to_store()
# servo.detect_ring_LOW_from_HIGH()


# servo.detect_plate_ring_LOW()
# servo.from_plate_ring_LOW_to_plate()
# servo.put_from_store_to_plate()
# servo.grab_from_ground_to_store()
# servo.hold_maduo_from_detect_HIGH()
# servo.maduo_down()
# servo.from_store_to_ground()
# servo.task_over()
# time.sleep(1)
# servo.task_over()
# servo.FilterServo(1,3560,2400,70)
# servo.hold_maduo_from_detect_HIGH()
# servo.detect_ring_HIGH_from_store()
# time.sleep(1)
# servo.task_over()

# servo.FilterServo(2,5050,3400,0)
# servo.FilterServo(2,4650,3400,0)
# servo.FilterServo(2,5300,3400,0)



# servo.PWMServo(16, 86)
# servo.turn_storage(3)
# servo.PWMServo(16, 56)
# servo.initialize_arm()
# servo.detect_ring_HIGH_from_store()
# servo.detect_ring_LOW_from_store()
# servo.detect_LOW_to_ground()
# servo.detect_plate()
# servo.detect_plate_ring_LOW()
# servo.from_plate_ring_LOW_to_plate()
# servo.put_from_store_to_plate()
# servo.from_store_to_ground()
# servo.grab_from_ground_to_store()
# servo.detect_ring_LOW_from_ground()
# servo.detect_plate()
# servo.grab_from_plate()
# servo.grab_from_ground()
# servo.put_ground()
# # servo.put_plate()
# servo.hold_maduo_from_detect_HIGH()
# servo.detect_ring_HIGH_from_store()
# servo.maduo_down()
# servo.task_over()
# servo.PWMServo(16, 58)
# time.sleep(0.3)
# servo.FilterServo(2,3950,3400,0)
# time.sleep(0.2)
# servo.PWMServo(16, 56)
# servo.put_from_store_to_plate()
# servo.from_plate_ring_LOW_to_plate()
# servo.put_from_LOW_to_ground()
# servo.PWMServo(16, 30)
# servo.FilterServo(2,2160,3400,0)
# time.sleep(3)
# servo.PWMServo(16, 86)
# servo.FilterServo(2,2200,3400,0)
# servo.FilterServo(2,5680,3400,0)
# time.sleep(0.8)
# servo.FilterServo(1,3620,2800,200)
# servo.FilterServo(1,2048,2800,70)
# time.sleep(0.35)

# servo.FilterServo(2,2400,3400,200) 

