import cv2
from Cameracontrol import Cameracontrol
from Servocontrol import Servocontrol
import time

def test_get_center():
    try:
        color_cam = Cameracontrol(0)
        
        while True:
            try:
                # 测试 get_center 功能
                color_coords_dict, frame, found_flag, still_flag = color_cam.get_center(
                    duration=0.05,
                    still_threshold=2,
                    MIN_AREA=13000,  
                    MAX_AREA=38000
                )
                
                # 打印输出结果
                print(f"found_flag: {found_flag}")
                print(f"still_flag: {still_flag}")
                for color, coords in color_coords_dict.items():
                    print(f"{color}: {coords}")
                
                # 显示处理后的图像
                if frame is not None:
                    cv2.imshow("Test Frame", frame)
                
                # 按 'q' 退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("quit")
                    break
                    
                # 添加短暂延时
                time.sleep(0.1)
                
            except Exception as e:
                print(f"error: {e}")
                break
                
    except Exception as e:
        print(f"error: {e}")
        
    finally:
        # 释放资源
        if 'color_cam' in locals():
            color_cam.close_windows()
            color_cam.close_cam()

if __name__ == "__main__":
    test_get_center()