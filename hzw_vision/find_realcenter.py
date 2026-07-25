import cv2
import numpy as np
import time
# 测量面积半径，找到机械臂机械中心

def draw_circle_on_frame(frame, area, center_x, center_y):
    # 计算圆的半径
    radius = int(np.sqrt(area / np.pi))

    # 在图像上绘制圆
    cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 1)

def main():
    # 记录初始化开始时间
    start_time = time.time()

    # 创建摄像头对象
    try:
        cap = cv2.VideoCapture(0,cv2.CAP_V4L2) 
        if not cap.isOpened():
            raise ValueError("camera not open")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) 
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 

        # 记录初始化结束时间
        end_time = time.time()

        # 计算并打印初始化时间
        init_time = end_time - start_time
        print(f"Camera init success, time taken: {init_time:.4f} seconds")
        for _ in range(5):
            ret, _ = cap.read()
            time.sleep(0.1)
    except Exception as e:
        print(f"camera init error: {e}")
        raise

    # 创建一个窗口
    cv2.namedWindow('Camera')
    
    # 初始值
    radius = 70
    center_x = 315
    center_y = 176
    circle_area = np.pi * radius * radius
    
    # 创建滑动条
    cv2.createTrackbar('Radius', 'Camera', radius, 200, lambda x: None)
    cv2.createTrackbar('Center X', 'Camera', center_x, 640, lambda x: None)
    cv2.createTrackbar('Center Y', 'Camera', center_y, 480, lambda x: None)
    cv2.createTrackbar('Area', 'Camera', int(circle_area), 100000, lambda x: None)
    
    # 创建切换模式的滑动条（0：通过半径控制，1：通过面积控制）
    cv2.createTrackbar('Mode', 'Camera', 0, 1, lambda x: None)

    try:
        while True:
            # 获取当前帧
            ret, frame = cap.read()
            if not ret:
                print("无法获取视频帧")
                break
                
            # 获取滑动条的值
            mode = cv2.getTrackbarPos('Mode', 'Camera')
            
            if mode == 0:  # 通过半径控制
                radius = cv2.getTrackbarPos('Radius', 'Camera')
                # 更新面积滑动条
                circle_area = int(np.pi * radius * radius)
                cv2.setTrackbarPos('Area', 'Camera', circle_area)
            else:  # 通过面积控制
                circle_area = cv2.getTrackbarPos('Area', 'Camera')
                # 更新半径滑动条
                radius = int(np.sqrt(circle_area / np.pi))
                cv2.setTrackbarPos('Radius', 'Camera', radius)
            
            center_x = cv2.getTrackbarPos('Center X', 'Camera')
            center_y = cv2.getTrackbarPos('Center Y', 'Camera')
            
            # 在帧上绘制圆
            cv2.circle(frame, (center_x, center_y), radius, (0, 255, 0), 1)
            
            # 显示圆的信息
            cv2.putText(frame, f"Radius: {radius}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Area: {circle_area}px^2", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Center: ({center_x}, {center_y})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 绘制十字线
            cross_length = 5
            cv2.line(frame, (int(center_x - cross_length), int(center_y)),
                    (int(center_x + cross_length), int(center_y)), (0, 0, 255), 1)
            cv2.line(frame, (int(center_x), int(center_y - cross_length)),
                    (int(center_x), int(center_y + cross_length)), (0, 0, 255), 1)   
            
            # 显示帧
            cv2.imshow('Camera', frame)

            # 按下 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
