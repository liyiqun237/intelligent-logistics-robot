import cv2
import numpy as np
import serial
import time
import struct
from pyzbar.pyzbar import decode
from collections import Counter
import threading
import subprocess

class Cameracontrol:
    def __init__(self,cam_mode):
        """
        初始化Cameracontrol类的实例。

        参数:
            cam_mode (int): 摄像头类型，0为颜色摄像头，1为扫码摄像头。

       """
        self.frame=None
        self.prev_frame_time = 0
        self.curr_frame_time = 0
        self.prev_centers = []#圆环中心
        self.center_x =312#机械臂中心x坐标
        self.center_y =176#机械臂中心y坐标
        self.last_theta = 0
        if cam_mode == 0:
            try:

                self.cap = cv2.VideoCapture(0,cv2.CAP_V4L2) #V4l2树莓派
                if not self.cap.isOpened():
                    raise ValueError("camera not open")
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) 
                # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  
                for _ in range(5):
                    ret, _ = self.cap.read()
                    time.sleep(0.1)
            except Exception as e:
                print(f"camera init error: {e}")
                raise

            # for camera_index in range(1,10):
            #     self.cap = cv2.VideoCapture(camera_index,cv2.CAP_V4L2)
            #     self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

            #     if self.cap.isOpened():
            #         print(f"成功打开color摄像头,序号为: {camera_index}")
            #         break
            #     else:
            #         self.cap.release()
            # else:
            #     raise ValueError("没有找到可以打开的摄像头设备")
            
        elif(cam_mode ==1):

            self.cap = cv2.VideoCapture(2,cv2.CAP_V4L2) 
            if not self.cap.isOpened():
                raise ValueError("camera not open")
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) 
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
            


    
    
    
    def close_cam(self):
        self.cap.release()
    
    
    
    def close_windows(self):
        """
        释放摄像头资源，并关闭相关的图像显示窗口。
        """
        cv2.destroyAllWindows()
    
    
    
    def update_color_center(self,color):
        """
        更新目标颜色中心的坐标。
        参数：
            color(int)
        返回值：(x,y)
        """
        color_coords, found_flag, _ = self.get_color_center(color,duration=0.05,still_threshold=1,MIN_AREA=13000,MAX_AREA=40000)#30000
        if(found_flag):
            #更新颜色坐标
            return color_coords
        else:
            return None,None
    
    
    
    def non_max_suppression(slef,boxes, scores, threshold):
        """
        非极大值抑制函数

        参数:
        boxes (numpy.ndarray): 边界框数组，形状为 (N, 4)，其中N是边界框的数量，每个边界框的格式为 [x1, y1, x2, y2]
        scores (numpy.ndarray): 对应边界框的得分数组，形状为 (N,)
        threshold (float): 抑制的阈值，当重叠度大于该阈值时，抑制得分较低的框

        返回:
        keep (list): 保留的边界框索引列表
        """
        if len(boxes) == 0:
            return []

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = scores.argsort()[::-1]

        keep = []
        while len(order) > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)

            overlap = (w * h) / areas[order[1:]]

            inds = np.where(overlap <= threshold)[0]
            order = order[inds + 1]

        return keep    
   
   
   
    def apply_temporal_filter(self,current_results,smooth_factor=0.4):
        """
        应用时间滤波器，对当前检测结果进行平滑处理。
        
        参数:
        current_results (list): 当前检测结果列表，每个元素为(x, y, r)
        
        返回:
        list: 经过时间滤波处理后的检测结果列表
        """
    
        if not self.prev_centers:  # 首次检测
            self.prev_centers = current_results.copy()
            return current_results
    
        filtered_results = []
    
        # 为每个当前检测结果找到对应的历史结果
        for curr_x, curr_y, curr_r in current_results:
            # 查找最近的历史点
            best_match = None
            min_dist = float('inf')
            
            for i, (prev_x, prev_y, prev_r) in enumerate(self.prev_centers):
                dist = ((curr_x - prev_x)**2 + (curr_y - prev_y)**2)**0.5
                if dist < min_dist:
                    min_dist = dist
                    best_match = (i, prev_x, prev_y, prev_r)
            
            # 如果找到匹配点且距离合理，应用平滑
            if best_match and min_dist < curr_r*0.25:  # 阈值可调整
                i, prev_x, prev_y, prev_r = best_match
                # 指数平滑
                smooth_x = smooth_factor * prev_x + (1 - smooth_factor) * curr_x
                smooth_y = smooth_factor * prev_y + (1 - smooth_factor) * curr_y
                smooth_r = smooth_factor * prev_r + (1 - smooth_factor) * curr_r
                
                filtered_results.append((smooth_x, smooth_y, smooth_r))
                # 更新历史点
                self.prev_centers[i] = (smooth_x, smooth_y, smooth_r)
            else:
                # 新检测点，直接添加
                filtered_results.append((curr_x, curr_y, curr_r))
                self.prev_centers.append((curr_x, curr_y, curr_r))
        
        # 移除未匹配的历史点
        if filtered_results:
            new_prev_centers = []
            for prev_point in self.prev_centers:
                for curr_point in filtered_results:
                    px, py, pr = prev_point
                    cx, cy, cr = curr_point
                    if ((px - cx)**2 + (py - cy)**2)**0.5 < cr*0.25:
                        new_prev_centers.append(prev_point)
                        break
            self.prev_centers = new_prev_centers
        
        return filtered_results
    
    
    
    def get_angle(self):
        """
        从摄像头获取图像帧，检测图像中的直线并返回角度信息。

        返回:
            float: 经过处理后的直线角度信息（单位：度）。
        """
        ret, frame = self.cap.read()
        ret, frame = self.cap.read()  # 读取两次确保获取到最新的一帧

        cnt_line = 0
        gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=150)  # 获取图中的直线
        cnt = 0
        sumTheta = 0
        averageTheta = 0
        if lines is not None:
            for line in lines:  # 遍历每一条线
                rho, theta = line[0]
                if ((np.abs(theta) >= 1.1) & (np.abs(theta) <= 2.2)):  # 选择一定角度范围的线（弧度）
                    cnt = cnt + 1
                    sumTheta = sumTheta + theta / 5.0
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))
                    cv2.line(gray, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 绘制线条
        if not (cnt == 0):
            averageTheta = 5.0 * sumTheta / cnt  # 计算角度的平均值
            self.last_theta = averageTheta
        else:
            averageTheta = self.last_theta

        # cv2.imshow("line", gray)  # 显示处理后的图像
        # cv2.waitKey(1)
        return (averageTheta / np.pi) * 180 - 90  # 返回角度，转换为度并调整偏移量
   
   
   
    def get_Qr(self):
        """
        使用类中初始化的摄像头开启扫码，返回识别到的二维码字符串数据。

        返回:
            str: 识别到的二维码字符串数据，如果未识别到则返回None。
        """   
        print("识别二维码----")
        while True:
            ret,frame = self.cap.read()
            alpha = 1.2  # 调整对比度
            beta = 10    # 调整亮度
            light_frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
            # 使用pyzbar库解码图像中的条码信息    
            if not ret or frame is None:
                print("Failed to grab frame")
                return None, None, None, 0 
            gray = cv2.cvtColor(light_frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = cv2.adaptiveThreshold(blurred, 255, 
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY, 11, 2)     
            decoded_objects = decode(frame)
            if not decoded_objects:
                decoded_objects = decode(gray)
            if not decoded_objects:
                decoded_objects = decode(thresh)
            if decoded_objects:
                decoded_objects = decode(light_frame)
            # 遍历解码得到的条码对象            
            for obj in decoded_objects:
                data = obj.data.decode('utf-8')
                self.close_windows()
                # 如果成功解码到条码，则返回识别到的字符串数据
                return data
            # 查看扫码时的实时画面
            cv2.imshow('getting_Qr', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        return None
    

##----------------------------------------色块-------------------------------------------------
    def get_center(self, duration=0.5, still_threshold=2, MIN_AREA=16000, MAX_AREA=38000):
        """
        得到颜色块的坐标和静止状态
        只适用于视野里每种颜色只有一个色块的情况，若每种颜色出现多个，则需要聚类
        返回：
            颜色坐标字典，画了框的图像，found_flag, still_flag
        """
        # 颜色阈值定义
        dim_red_min = [0, 60, 60]
        dim_red_max = [13, 255, 255]
        dim_red_min1 = [160, 50, 50]
        dim_red_max1 = [180, 255, 255]
        dim_green_min = [39, 39, 50]
        dim_green_max = [95, 255, 255]
        dim_blue_min = [100, 60, 60]
        dim_blue_max = [124, 255, 255]

        start_time = time.time()
        color_coords_dict = {
            "red": [],
            "green": [],
            "blue": []
        }
        ret, frame = self.cap.read()

        if not ret or frame is None:
            print("Failed to grab frame_1")
            return None, None, None, 0     
        while time.time() - start_time < duration:
            ret, frame = self.cap.read()

            if not ret or frame is None:
                print("Failed to grab frame_2")
                return None, None, None, 0

            # frame = cv2.resize(frame, (640, 480))  # 设置分辨率
            # (b,g,r) = cv2.split(frame)
            # equal_b = cv2.equalizeHist(b)
            # equal_g = cv2.equalizeHist(g)
            # equal_r = cv2.equalizeHist(r)
            # frame = cv2.merge((equal_b,equal_g,equal_r))
            # frame = cv2.GaussianBlur(frame, (5, 5), 0)
            
            # result = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            # avg_a = np.average(result[:, :, 1])
            # avg_b = np.average(result[:, :, 2])
            # result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
            # result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
            # frame =  cv2.cvtColor(result, cv2.COLOR_LAB2BGR)

            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
            cl = clahe.apply(l)
            enhanced = cv2.merge((cl,a,b))
            frame = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
            # frame = cv2.medianBlur(frame, 3)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 创建颜色掩膜并处理红色物块
            red_mask1 = cv2.inRange(hsv, np.array(dim_red_min), np.array(dim_red_max))
            red_mask2 = cv2.inRange(hsv, np.array(dim_red_min1), np.array(dim_red_max1))
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)

            # 应用高斯滤波
            red_blurred = cv2.GaussianBlur(red_mask, (5, 5), 0)

            # 应用形态学操作增强
            red_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            red_opened = cv2.morphologyEx(red_blurred, cv2.MORPH_OPEN, red_kernel)
            red_closed = cv2.morphologyEx(red_opened, cv2.MORPH_CLOSE, red_kernel)
            # 查找红色物块轮廓
            red_contours, _ = cv2.findContours(red_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_NONE)
            for cnt in red_contours:
                area = cv2.contourArea(cnt)
                # print("red_area:",area)
                # 计算轮廓的周长
                if (MIN_AREA < area < MAX_AREA) :  # 过滤小和大的轮廓，并确保轮廓接近圆形
                    x, y, w, h = cv2.boundingRect(cnt)
                    if(1.4>(w/h)>0.7):
                        x_center = (x + w / 2) / 640
                        y_center = (y + h / 2) / 480
                        color_coords_dict["red"].append((x_center, y_center))

                        # 绘制轮廓而不是矩形框
                        cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 2)


            # 创建颜色掩膜并处理绿色物块
            green_mask = cv2.inRange(hsv, np.array(dim_green_min), np.array(dim_green_max))
            # 应用高斯滤波
            # green_blurred = cv2.GaussianBlur(green_mask, (5, 5), 0)
            #双边滤波
            # bilateral_filtered_img = cv2.bilateralFilter(green_mask, 5, 20, 75)
            kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
            sharpened_image = cv2.filter2D(green_mask, -1, kernel)
            # 应用形态学操作增强
            green_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            # green_kernel_2 = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))

            # green_opened = cv2.morphologyEx(sharpened_image, cv2.MORPH_OPEN, green_kernel)
            green_closed = (cv2.morphologyEx(sharpened_image, cv2.MORPH_CLOSE, green_kernel))
            # green_closed = (cv2.morphologyEx(green_closed, cv2.MORPH_CLOSE, green_kernel_2))
            # 查找绿色物块轮廓
            green_contours, _ = cv2.findContours(green_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_NONE)

            green_boxes = []
            green_scores = []
            for cnt in green_contours:
                area = cv2.contourArea(cnt)
                # print("green_area:",area)
                if MIN_AREA < area < MAX_AREA:  # 过滤小和大的轮廓
                    x, y, w, h = cv2.boundingRect(cnt)
                    if(1.4>(w/h)>0.7):
                        x_center = (x + w / 2) / 640
                        y_center = (y + h / 2) / 480
                        color_coords_dict["green"].append((x_center, y_center))
                        cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 2)
                    


            # 创建颜色掩膜并处理蓝色物块
            blue_mask = cv2.inRange(hsv, np.array(dim_blue_min), np.array(dim_blue_max))

            # 应用高斯滤波
            blue_blurred = cv2.GaussianBlur(blue_mask, (5, 5), 0)

            # 应用形态学操作增强
            blue_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            blue_opened = (cv2.morphologyEx(blue_blurred, cv2.MORPH_OPEN, blue_kernel))
            blue_closed = (cv2.morphologyEx(blue_opened, cv2.MORPH_CLOSE, blue_kernel))
            # cv2.imshow("blue", blue_closed)
            # cv2.imshow("red", red_closed)
            # cv2.imshow("green", green_closed)
            # 查找蓝色物块轮廓
            blue_contours, _ = cv2.findContours(blue_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_SIMPLE)
            blue_boxes = []
            blue_scores = []
            for cnt in blue_contours:
                area = cv2.contourArea(cnt)
                # print("blue_area:",area)

                if MIN_AREA < area < MAX_AREA:  # 过滤小和大的轮廓
                    x, y, w, h = cv2.boundingRect(cnt)
                    if(1.4>(w/h)>0.7):
                        x_center = (x + w / 2) / 640
                        y_center = (y + h / 2) / 480
                        color_coords_dict["blue"].append((x_center, y_center))
                        cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 2)
                    


        # 用于存储每种颜色坐标平均值的字典
        average_coords_dict = {}
        for color, coords in color_coords_dict.items():
            if coords:
                # 分别提取x坐标和y坐标的列表
                x_coords = [coord[0] for coord in coords]
                y_coords = [coord[1] for coord in coords]
                # 计算x坐标和y坐标的平均值
                avg_x = sum(x_coords) / len(x_coords)
                avg_y = sum(y_coords) / len(y_coords)
                average_coords_dict[color] = (avg_x, avg_y)


        # 判断有没有识别到
        found_flag = 0
        for color, coords in color_coords_dict.items():
            if coords:
                found_flag = 1
                break

        # 统计每种颜色物块坐标的出现次数
        for color, coords in color_coords_dict.items():
            if coords:
                coord_counter = Counter(coords)
                color_coords_dict[color] = coord_counter.most_common()

        # 判断物块是否静止
        still_flag = 0
        for color, coord_count_list in color_coords_dict.items():
            if coord_count_list:
                most_common_coord, count = coord_count_list[0]
                if count >= still_threshold:
                    still_flag = 1
                    break
        self.curr_frame_time = time.time()
        fps = 1 / (self.curr_frame_time - self.prev_frame_time)
        self.prev_frame_time = self.curr_frame_time
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return average_coords_dict, frame, found_flag, still_flag
    def get_color_blocks_sorted_by_x(self, duration=0.5, still_threshold=2, MIN_AREA=16000, MAX_AREA=38000):
        """
        获取视野中的颜色块并按照x坐标（左右顺序）排序
        
        参数:
        duration (float): 采集图像帧的持续时间
        still_threshold (int): 判断物块静止的阈值
        MIN_AREA (int): 色块面积的最小值
        MAX_AREA (int): 色块面积的最大值
        
        返回:
        sorted_colors (list): 按照从左到右顺序排列的颜色ID列表 [1=红, 2=绿, 3=蓝]
        found_flag (int): 是否检测到色块
        still_flag (int): 色块是否静止
        """
        color_coords_dict, frame, found_flag, still_flag = self.get_center(duration, still_threshold, MIN_AREA, MAX_AREA)
        
        if not found_flag or not color_coords_dict:
            return [], found_flag, still_flag
        
        # 创建颜色ID与坐标的映射
        color_id_mapping = {
            "red": 1,
            "green": 2,
            "blue": 3
        }
        
        # 创建包含颜色ID和x坐标的列表
        color_positions = []
        for color_name, coords in color_coords_dict.items():
            if coords:  # 确保有坐标
                color_id = color_id_mapping[color_name]
                x_coord = coords[0]  # x坐标是元组的第一个元素
                color_positions.append((color_id, x_coord))
        
        # 按x坐标排序（从左到右）
        sorted_colors = [color_id for color_id, _ in sorted(color_positions, key=lambda item: item[1])]
        
        # 在图像上显示排序结果
        if frame is not None:
            sorted_text = f"排序: {sorted_colors}"
            cv2.putText(frame, sorted_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, (255, 255, 0), 2)
            cv2.imshow("Sorted Colors", frame)
            cv2.waitKey(1)
        
        return sorted_colors, found_flag, still_flag


    def get_every_color_center(self, duration=0.5, still_threshold=2, MIN_AREA=15000, MAX_AREA=39000, distance_threshold=0.1):
        """
        获取视野中每个色块的中心坐标
        每种颜色可能有多个色块，得到每个色块的中心坐标
        
        参数:
        duration (float): 采集图像帧的持续时间，默认值为0.5秒
        
        返回:
        包含每种颜色的坐标列表、found_flag（是否检测到色块）、still_flag（是否有静止物块）
        """
        from sklearn.cluster import KMeans  # 确保导入 KMeans

        # 颜色阈值定义
        dim_red_min = [0, 60, 60]
        dim_red_max = [13, 255, 255]
        dim_red_min1 = [160, 50, 50]
        dim_red_max1 = [180, 255, 255]
        dim_green_min = [39, 36, 50]
        dim_green_max = [95, 255, 255]
        dim_blue_min = [100, 60, 60]
        dim_blue_max = [124, 255, 255]

        start_time = time.time()
        color_coords_dict = {
            "red": [],
            "green": [],
            "blue": []
        }
        ret, frame = self.cap.read()

        if not ret or frame is None:
            print("Failed to grab frame_1")
            return None, None, None, 0     
        while time.time() - start_time < duration:
            ret, frame = self.cap.read()

            if not ret or frame is None:
                print("Failed to grab frame_2")
                return None, None, None, 0

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 创建颜色掩膜并处理红色物块
            red_mask1 = cv2.inRange(hsv, np.array(dim_red_min), np.array(dim_red_max))
            red_mask2 = cv2.inRange(hsv, np.array(dim_red_min1), np.array(dim_red_max1))
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)

            # 应用高斯滤波
            red_blurred = cv2.GaussianBlur(red_mask, (5, 5), 0)

            # 应用形态学操作增强
            red_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            red_opened = cv2.morphologyEx(red_blurred, cv2.MORPH_OPEN, red_kernel)
            red_closed = cv2.morphologyEx(red_opened, cv2.MORPH_CLOSE, red_kernel)

            # 查找红色物块轮廓
            red_contours, _ = cv2.findContours(red_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_NONE)
            for cnt in red_contours:
                area = cv2.contourArea(cnt)
                if (MIN_AREA < area < MAX_AREA):  # 过滤小和大的轮廓
                    x, y, w, h = cv2.boundingRect(cnt)
                    if(1.4>(w/h)>0.7):    
                        x_center = (x + w / 2) / 640
                        y_center = (y + h / 2) / 480
                        color_coords_dict["red"].append((x_center, y_center))

            # 创建颜色掩膜并处理绿色物块
            green_mask = cv2.inRange(hsv, np.array(dim_green_min), np.array(dim_green_max))
            green_blurred = cv2.GaussianBlur(green_mask, (5, 5), 0)
            green_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            green_opened = cv2.morphologyEx(green_blurred, cv2.MORPH_OPEN, green_kernel)
            green_closed = cv2.morphologyEx(green_opened, cv2.MORPH_CLOSE, green_kernel)

            # 查找绿色物块轮廓
            green_contours, _ = cv2.findContours(green_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_NONE)
            for cnt in green_contours:
                area = cv2.contourArea(cnt)
                if MIN_AREA < area < MAX_AREA:  # 过滤小和大的轮廓
                    x, y, w, h = cv2.boundingRect(cnt)
                    if(1.4>(w/h)>0.7):
                        x_center = (x + w / 2) / 640
                        y_center = (y + h / 2) / 480
                        color_coords_dict["green"].append((x_center, y_center))

            # 创建颜色掩膜并处理蓝色物块
            blue_mask = cv2.inRange(hsv, np.array(dim_blue_min), np.array(dim_blue_max))
            blue_blurred = cv2.GaussianBlur(blue_mask, (5, 5), 0)
            blue_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            blue_opened = cv2.morphologyEx(blue_blurred, cv2.MORPH_OPEN, blue_kernel)
            blue_closed = cv2.morphologyEx(blue_opened, cv2.MORPH_CLOSE, blue_kernel)

            # 查找蓝色物块轮廓
            blue_contours, _ = cv2.findContours(blue_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_SIMPLE)
            for cnt in blue_contours:
                area = cv2.contourArea(cnt)
                if MIN_AREA < area < MAX_AREA:  # 过滤小和大的轮廓
                    x, y, w, h = cv2.boundingRect(cnt)
                    if(1.4>(w/h)>0.7):
                        x_center = (x + w / 2) / 640
                        y_center = (y + h / 2) / 480
                        color_coords_dict["blue"].append((x_center, y_center))

        # 对每种颜色的坐标进行分组
        clustered_coords_dict = {}
        ave_clustered_coords_dict = {}
        for color, coords in color_coords_dict.items():
            if coords:
                clustered_coords = []
                current_cluster = []

                for (x, y) in coords:
                    if not current_cluster:
                        current_cluster.append((x, y))
                    else:
                        # 计算当前坐标与当前簇中最后一个坐标的距离
                        last_x, last_y = current_cluster[-1]
                        distance = (x - last_x) ** 2 + (y - last_y) ** 2  # 使用平方距离避免开方运算

                        if distance <= distance_threshold ** 2 and len(clustered_coords) < 2:
                            current_cluster.append((x, y))
                        else:
                            clustered_coords.append(current_cluster)
                            current_cluster = [(x, y)]  # 开始新的簇

                # 添加最后一个簇
                if current_cluster:
                    clustered_coords.append(current_cluster)

                # 限制最大类数
                clustered_coords_dict[color] = clustered_coords[:2]  # 只保留最多 2 个簇
                # 计算每个簇的均值
                mean_coords = []
                for cluster in clustered_coords:
                    if cluster:  # 确保簇不为空
                        mean_x = sum(coord[0] for coord in cluster) / len(cluster)
                        mean_y = sum(coord[1] for coord in cluster) / len(cluster)
                        mean_coords.append((mean_x, mean_y))

                ave_clustered_coords_dict[color] = mean_coords  # 更新为均值坐标

        # 判断有没有识别到
        found_flag = any(len(coords) > 0 for coords in color_coords_dict.values())

        # 判断物块是否静止
        still_flag = 0
        for color, coords in color_coords_dict.items():
            if coords:
                coord_counter = Counter(coords)
                most_common_coords = coord_counter.most_common()
                
                # 检查是否有任何坐标出现的次数超过阈值
                for coord, count in most_common_coords:
                    if count >= still_threshold:
                        still_flag = 1
                        break
                
                # 如果已经确定有静止的色块，就不需要继续检查
                if still_flag == 1:
                    break

        self.curr_frame_time = time.time()
        fps = 1 / (self.curr_frame_time - self.prev_frame_time)
        self.prev_frame_time = self.curr_frame_time
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return ave_clustered_coords_dict, frame, found_flag, still_flag



    def get_closest_center(self, target_coord, duration=0.5,still_threshold=2, MIN_AREA=15000, MAX_AREA=39000,Mode ="single"):
        """
        得到离目标位置最近的色块颜色和坐标
        single模式只找单个色块,every模式找每个色块
        返回：
            颜色(int),坐标(x,y),found_flag,still_flag
        """
        color_mapping = {
            "red":1,
            "green": 2,
            "blue": 3
        }
        if Mode == "single":
            color_coords_dict, frame, found_flag, still_flag = self.get_center(duration, still_threshold, MIN_AREA, MAX_AREA)
        elif Mode == "every":
            color_coords_dict, frame, found_flag, still_flag = self.get_every_color_center(duration, still_threshold, MIN_AREA, MAX_AREA)
        closest_color = None
        cv2.imshow("cloest",frame)
        cv2.waitKey(1)
        closest_distance = float('inf')
        if found_flag:
            for color, coord in color_coords_dict.items():
                distance = np.sqrt((coord[0] - target_coord[0]) ** 2 + (coord[1] - target_coord[1]) ** 2)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_color = color
            if closest_color:
                # self.close_windows()
                return color_mapping.get(closest_color), color_coords_dict[closest_color], found_flag, still_flag
                
        else:
            return None, None, found_flag, still_flag



    def get_closer_to_car_center(self, duration=0.5, still_threshold=2, MIN_AREA=15000, MAX_AREA=39000, Mode="single"):
        """
        获取 y 值最小的色块颜色和坐标（即最靠近屏幕上方的坐标）
        
        参数:
        duration (float): 采集图像帧的持续时间，默认值为0.5秒
        still_threshold (int): 判断物块静止的阈值，默认值为2
        MIN_AREA (int): 色块面积的最小值，默认值为15000
        MAX_AREA (int): 色块面积的最大值，默认值为39000
        Mode (str): 检测模式，"single"表示只检测单个色块，"every"表示检测每个色块
        
        返回:
        颜色(int), 坐标(x,y), found_flag, still_flag
        """
        color_mapping = {
            "red": 1,
            "green": 2,
            "blue": 3
        }
        
        if Mode == "single":
            color_coords_dict, frame, found_flag, still_flag = self.get_center(duration, still_threshold, MIN_AREA, MAX_AREA)
        elif Mode == "every":
            color_coords_dict, frame, found_flag, still_flag = self.get_every_color_center(duration, still_threshold, MIN_AREA, MAX_AREA)
        
        min_y_color = None
        min_y_coord = None
        min_y_value = float('inf')
        if(frame is not None):
            # 在图像上标注stillflag状态
            flag_text = f"StillFlag: {still_flag}"
            cv2.putText(frame, flag_text, (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       1, (0, 255, 0), 2)  # 绿色文字
            cv2.imshow("closest", frame)
            cv2.waitKey(1)
        if found_flag:
            for color, coords in color_coords_dict.items():
                if isinstance(coords, list):  # 处理 get_every_color_center 返回的列表
                    for coord in coords:
                        if coord[1] < min_y_value:  # 比较 y 值
                            min_y_value = coord[1]
                            min_y_color = color
                            min_y_coord = coord
                else:  # 处理 get_center 返回的单个坐标
                    if coords[1] < min_y_value:  # 比较 y 值
                        min_y_value = coords[1]
                        min_y_color = color
                        min_y_coord = coords
            
            if min_y_color:
               
                return color_mapping.get(min_y_color), min_y_coord, found_flag, still_flag
        
        return None, None, found_flag, still_flag



    def get_color_center(self, color_id, duration=0.5, still_threshold=2, MIN_AREA=16000, MAX_AREA=38000):
        """
        获取指定颜色块的中心坐标及相关标志

        参数:
        color_id (int): 指定的颜色标识，取值为1（表示红色）、2（表示绿色）、3（表示蓝色）
        duration (float): 采集图像帧的持续时间，默认值为0.5秒
        still_threshold (int): 判断物块静止的阈值，默认值为2

        返回:
        包含指定颜色的坐标列表（如果检测到指定颜色）、found_flag（是否检测到指定颜色）、still_flag（是否有静止物块）
        """
        color_coords_dict, frame, found_flag, still_flag = self.get_center(duration, still_threshold, MIN_AREA, MAX_AREA)
        color_mapping = {
            1: "red",
            2: "green",
            3: "blue"
        }
        target_color = color_mapping.get(color_id)
        target_coords = []
        if(frame is not None):

            if found_flag:
            # 检查映射后的指定颜色是否在字典中且有对应的坐标数据
                if target_color in color_coords_dict and color_coords_dict[target_color]:
                    target_coords = color_coords_dict[target_color]
                    found_flag = 1
                    cross_length = 5
                    cv2.line(frame, (int(target_coords[0] * 640 - cross_length), int(target_coords[1] * 480)),
                    (int(target_coords[0] * 640 + cross_length), int(target_coords[1]  * 480)), (0, 0, 255), 1)
                    cv2.line(frame, (int(target_coords[0]  * 640), int(target_coords[1]  * 480 - cross_length)),
                    (int(target_coords[0]  * 640), int(target_coords[1]  * 480 + cross_length)), (0, 0, 255), 1)  
                else:
                    found_flag = 0
            cv2.imshow("colorcenter",frame)
            cv2.waitKey(1)

        return target_coords, found_flag, still_flag
    

##----------------------------------------圆环-------------------------------------------------
    def get_ring_center(self,frame,r_min=70,r_max=100,smooth_factor=0.4):
        """
        获取圆环的中心坐标
        
        参数:
        frame : 输入的图像帧
        r_min (int): 圆环内径的最小半径
        r_max (int): 圆环外径的最大半径

        返回:
        圆环的中心坐标(x,y,r)
        """
        median = cv2.medianBlur(frame,3)
        grayImg = cv2.cvtColor(median,cv2.COLOR_BGR2GRAY)
        # cv2.imshow("grayImg",grayImg)
        grayImg = cv2.GaussianBlur(grayImg,(5,5),0)
        cannyImg = cv2.Canny(grayImg,50,150)
        #cv2.imshow("cannyImg",cannyImg)
        circle_size=0
        pre_list=[]
        circles_list=[]
        res_list=[]

        circles_pre = cv2.HoughCircles(grayImg, cv2.HOUGH_GRADIENT_ALT, 1.5, 80, 
                                       param1=240, param2=0.80, 
                                       minRadius=r_min, maxRadius=r_max)
        # 检查第一次检测结果
        if circles_pre is None:
            # print("未检测到初始圆环")
            return []
        
        circles_pre = np.round(circles_pre[0, :]).astype("int")
        pre_list = [(x, y, r) for (x, y, r) in circles_pre]
        circle_size = len(pre_list)
        
        
        # 第二次检测
        circles = cv2.HoughCircles(grayImg, cv2.HOUGH_GRADIENT_ALT, 1.8, 30, 
                                 param1=340, param2=0.95, 
                                minRadius=r_min, maxRadius=r_max)
        if circles is None:
            # print("未检测到精细圆环")
            return []
        
        circles = np.round(circles[0, :]).astype("int")
        
        # 初始化分类列表
        circles_list = [[] for _ in range(circle_size)]
        
        # 匹配逻辑
        for (x, y, r) in circles:
            if circle_size > 0:
                distances = [(i, (x-pre_x)**2 + (y-pre_y)**2) for i, (pre_x, pre_y, _) in enumerate(pre_list)]
                if distances:
                    closest_idx, closest_dist = min(distances, key=lambda item: item[1])
                    # 只有当距离小于阈值时才匹配
                    if closest_dist < (r * 0.5)**2:  # 距离阈值为半径的一半
                        circles_list[closest_idx].append((x, y, r))
                    else:
                        # 距离太远，可能是新的圆，创建新分组
                        if len(circles_list) < 4:  # 限制最大分组数
                            circles_list.append([(x, y, r)])
                            circle_size += 1
        
        # 滤波得到精细坐标
        for i in range(circle_size):
            x_sum = 0
            y_sum = 0
            r_sum = 0  # 添加半径求和
            size = len(circles_list[i])
            
            for x, y, r in circles_list[i]:
                # cv2.circle(frame, (x, y), r, (0, 255, 0), 1)
                x_sum += x
                y_sum += y
                r_sum += r  # 累加半径
            
            if(size != 0):
            # 使用中值滤波而非平均值
                # 在计算圆心时
                if size >= 3:  # 至少需要3个点才能计算中值
                    # 分别提取x,y,r坐标列表
                    x_list = [x for x,_,_ in circles_list[i]]
                    y_list = [y for _,y,_ in circles_list[i]]
                    r_list = [r for _,_,r in circles_list[i]]
                    
                    # 计算中值
                    x_med = sorted(x_list)[len(x_list)//2]
                    y_med = sorted(y_list)[len(y_list)//2]
                    r_med = sorted(r_list)[len(r_list)//2]
                    
                    res_list.append((x_med, y_med, r_med))

                else:
                    # 点太少，使用平均值
                    x_avg = x_sum / size
                    y_avg = y_sum / size
                    r_avg = r_sum / size
                    res_list.append((x_avg, y_avg, r_avg))
                

                


        # 在返回结果前应用时间滤波
        res_list = self.apply_temporal_filter(res_list,smooth_factor)

        # 修改顶部坐标显示，增加半径信息
        if len(res_list) > 0:
            coords_text = " "
            for idx, (x, y, r) in enumerate(res_list):  # 注意这里增加了r
                coords_text += f"({x:.2f},{y:.2f},R={r:.2f})"
                if idx < len(res_list) - 1:
                    coords_text += ", "
            
            cv2.putText(frame, coords_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (0, 255, 255), 2)
        return res_list



    def get_closest_ring_center(self,target_coord,r_min=70,r_max=100,smooth_factor=0.4):
        """
        获取离目标位置最近的圆环中心坐标
        
        参数:
        target_coord (tuple): 目标位置的坐标 (x, y)
        r_min (int): 圆环内径的最小半径
        r_max (int): 圆环外径的最大半径
        smooth_factor (float): 时间滤波的平滑因子
        
        返回:
        found_flag (bool): 是否检测到圆环
        离目标位置最近的圆环中心坐标(x,y)
        """
        ret, frame = self.cap.read()

        if not ret or frame is None:
            print("Failed to grab frame")
            return None, None
        ring_coords = self.get_ring_center(frame,r_min,r_max,smooth_factor)

        if not ring_coords:
            print("未检测到圆环")
            return 0, None
        
        closest_coord = None
        closest_distance = float('inf')
        for coord in ring_coords:
            distance = np.sqrt((coord[0] - target_coord[0]) ** 2 + (coord[1] - target_coord[1]) ** 2)
            if distance < closest_distance:
                closest_distance = distance
                closest_coord = coord
        return 1,closest_coord

    def get_plate_color_ring_center(self, color_id, duration=0.5, still_threshold=2, MIN_AREA=15000, MAX_AREA=39000, r_min=70, r_max=100, smooth_factor=0.4):
        """
        获取指定颜色的圆环中心坐标
        这个版本可以判断是否静止，适用于靶子在转盘上
        
        参数:
        color_id (int): 指定的颜色标识，取值为1（表示红色）、2（表示绿色）、3（表示蓝色）
        duration (float): 采集图像帧的持续时间，默认值为0.5秒
        MIN_AREA (int): 色块面积的最小值，默认值为15000
        MAX_AREA (int): 色块面积的最大值，默认值为39000
        r_min (int): 圆环内径的最小半径
        r_max (int): 圆环外径的最大半径
        smooth_factor (float): 时间滤波的平滑因子
        
        返回:
        圆环的中心坐标(x,y,r)、found_flag（是否检测到圆环）,still_flag
        """
        # 颜色映射
        color_mapping = {
            1: "red",
            2: "green",
            3: "blue"
        }
        
        detected_color_id, target_coords, found_flag, still_flag = self.get_closer_to_car_center(duration, still_threshold, MIN_AREA, MAX_AREA,"single")
        
        # 检查检测到的颜色是否与目标颜色一致
        if found_flag and detected_color_id != color_id:
            print(f"检测到颜色ID为{detected_color_id}的色块，但目标颜色ID为{color_id}")
            # return None, 0, still_flag
        
        if not found_flag:
            print(f"未检测到颜色ID为{color_id}的色块")
            # return None, 0, still_flag
        if not still_flag:
            print("物料在动，无法精调")
            return None, 0, still_flag
        
        if target_coords is None or len(target_coords) < 2:
            print("目标坐标无效")
            return None, 0, still_flag
        
        # 获取当前帧
        ret, frame = self.cap.read()
        if not ret or frame is None:
                print("Failed to grab frame")
                return None, 0, still_flag
        
        # 检测圆环
        ring_coords = self.get_ring_center(frame, r_min, r_max, smooth_factor)
        
        if not ring_coords:
            print("未检测到圆环")
            return None, 0, still_flag
        
        # 判断圆环中心是否在色块范围内
        color_ring_centers = []
        for ring_coord in ring_coords:
            ring_x, ring_y, ring_r = ring_coord
            
            # 检查圆环中心是否在任何一个色块范围内
            
                    # 计算圆环中心与色块中心的距离
            color_x=target_coords[0]
            color_y=target_coords[1]
            distance = np.sqrt((ring_x - color_x*640)**2 + (ring_y - color_y*480)**2)                
            # 如果距离小于一定阈值，认为圆环中心在色块范围内
            # 这里的阈值可以根据实际情况调整
            if distance < ring_r * 1.5:  # 使用圆环半径的1.5倍作为阈值
                color_ring_centers.append((ring_x, ring_y, ring_r))
                break
        
        # 显示结果
        if color_ring_centers:
            # 在图像上标记检测到的圆环
            for x, y, r in color_ring_centers:
                cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 1)
                cross_length = 5
                cross_thickness = 1
                cross_color = (0, 255, 0)
                
                # 水平线
                cv2.line(frame, (int(x) - cross_length, int(y)), (int(x) + cross_length, int(y)), cross_color, cross_thickness) 
                
                # 垂直线
                cv2.line(frame, (int(x), int(y) - cross_length), (int(x), int(y) + cross_length), cross_color, cross_thickness)
            
            cv2.imshow("Color Ring", frame)
            cv2.waitKey(1)
            
            # 返回第一个检测到的圆环中心
            if (color_ring_centers is not None):
                return color_ring_centers[0], 1, still_flag
        
        return None, 0, still_flag
    def get_precise_color_ring_center(self, color_id, MIN_AREA=15000, MAX_AREA=39000, r_min=70, r_max=100, smooth_factor=0.4):
        """
        获取指定颜色的精确圆环中心坐标
        适用于抓住物料进行精调的情况
        参数:
        color_id (int): 指定的颜色标识，取值为1（表示红色）、2（表示绿色）、3（表示蓝色）
        MIN_AREA (int): 色块面积的最小值，默认值为15000
        MAX_AREA (int): 色块面积的最大值，默认值为39000
        r_min (int): 圆环内径的最小半径
        r_max (int): 圆环外径的最大半径
        smooth_factor (float): 时间滤波的平滑因子
        
        返回:
        圆环的中心坐标(x,y)、found_flag（是否检测到指定颜色圆环） 
        """
        found_flag = 0
        cnt1 = 0
        while( (found_flag == 0) and (cnt1 <4)):#如果没找到某个颜色的圆就再找一遍，最多找4遍
        #
            cnt1 = cnt1+1
            ret,frame = self.cap.read()
            ret,frame = self.cap.read()
            ret,frame = self.cap.read()
            
            # 颜色阈值定义
            dim_red_min = [0, 60, 60]
            dim_red_max = [13, 255, 255]
            dim_red_min1 = [160, 50, 50]
            dim_red_max1 = [180, 255, 255]
            dim_green_min = [39, 39, 50]
            dim_green_max = [95, 255, 255]
            dim_blue_min = [100, 60, 60]
            dim_blue_max = [124, 255, 255]
            #框的最大范围
            x_min = 65535
            x_max = 0
            y_min = 65535
            y_max = 0
            num = 0#框的数量
            # 使用CLAHE增强图像
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
            cl = clahe.apply(l)
            enhanced = cv2.merge((cl,a,b))
            frame = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)


            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            if color_id == 1:
                red_mask1 = cv2.inRange(hsv, np.array(dim_red_min), np.array(dim_red_max))
                red_mask2 = cv2.inRange(hsv, np.array(dim_red_min1), np.array(dim_red_max1))
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                # 应用高斯滤波
                red_blurred = cv2.GaussianBlur(red_mask, (5, 5), 0)
                # 应用形态学操作增强
                red_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                red_opened = cv2.morphologyEx(red_blurred, cv2.MORPH_OPEN, red_kernel)
                red_closed = cv2.morphologyEx(red_opened, cv2.MORPH_CLOSE, red_kernel)
                # 查找红色物块轮廓
                red_contours, _ = cv2.findContours(red_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_NONE)
                for cnt in red_contours:
                    area = cv2.contourArea(cnt)
                    if (MIN_AREA < area < MAX_AREA) :  # 过滤小和大的轮廓，并确保轮廓接近圆形
                        x, y, w, h = cv2.boundingRect(cnt)
                        num = num + 1
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 将检测到的颜色框起来
                        cv2.putText(frame, f'color:{color_id}', (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                        if x < x_min :
                            x_min = x
                        if y < y_min :
                            y_min = y
                        if x + w > x_max:
                            x_max = x + w
                        if y + h > y_max:
                            y_max = y + h
                        if num ==  1:
                            x_min = x
                            x_max = x + w
                            y_max = y + h
                            y_min = y

            elif color_id == 2:
                green_mask = cv2.inRange(hsv, np.array(dim_green_min), np.array(dim_green_max))
            # 应用高斯滤波
                green_mask = cv2.GaussianBlur(green_mask, (3, 3), 0)
                #双边滤波
                # bilateral_filtered_img = cv2.bilateralFilter(green_mask, 5, 20, 75)
                kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
                sharpened_image = cv2.filter2D(green_mask, -1, kernel)
                # 应用形态学操作增强
                green_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
                # green_kernel_2 = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))

                # green_opened = cv2.morphologyEx(sharpened_image, cv2.MORPH_OPEN, green_kernel)
                green_closed = (cv2.morphologyEx(sharpened_image, cv2.MORPH_CLOSE, green_kernel))
                # green_closed = (cv2.morphologyEx(green_closed, cv2.MORPH_CLOSE, green_kernel_2))
                # 查找绿色物块轮廓
                green_contours, _ = cv2.findContours(green_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_NONE)
                for cnt in green_contours:
                    area = cv2.contourArea(cnt)
                    if MIN_AREA < area < MAX_AREA:  # 过滤小和大的轮廓
                        x, y, w, h = cv2.boundingRect(cnt)
                        num = num + 1
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 将检测到的颜色框起来
                        cv2.putText(frame, f'color:{color_id}', (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                        if x < x_min :
                            x_min = x
                        if y < y_min :
                            y_min = y
                        if x + w > x_max:
                            x_max = x + w   
                        if y + h > y_max:
                            y_max = y + h
                        if num ==  1:
                            x_min = x
                            x_max = x + w
                            y_max = y + h
                            y_min = y



            elif color_id == 3:
                blue_mask = cv2.inRange(hsv, np.array(dim_blue_min), np.array(dim_blue_max))

                # 应用高斯滤波
                blue_blurred = cv2.GaussianBlur(blue_mask, (5, 5), 0)

                # 应用形态学操作增强
                blue_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                blue_opened = (cv2.morphologyEx(blue_blurred, cv2.MORPH_OPEN, blue_kernel))
                blue_closed = (cv2.morphologyEx(blue_opened, cv2.MORPH_CLOSE, blue_kernel))
                # 查找蓝色物块轮廓
                blue_contours, _ = cv2.findContours(blue_closed, method=cv2.RETR_TREE, mode=cv2.CHAIN_APPROX_SIMPLE)
                for cnt in blue_contours:
                    area = cv2.contourArea(cnt)
                    if MIN_AREA < area < MAX_AREA:  # 过滤小和大的轮廓
                        x, y, w, h = cv2.boundingRect(cnt)
                        num = num + 1
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 将检测到的颜色框起来
                        cv2.putText(frame, f'color:{color_id}', (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                        if x < x_min :
                            x_min = x
                        if y < y_min :
                            y_min = y
                        if x + w > x_max:
                            x_max = x + w
                        if y + h > y_max:
                            y_max = y + h
                        if num ==  1:
                            x_min = x
                            x_max = x + w
                            y_max = y + h
                            y_min = y
     

                        
            # 检测圆环
            ring_coords = self.get_ring_center(frame, r_min, r_max, smooth_factor)
            if not ring_coords:
                print("未检测到圆环")
                return None, 0
            
            # 判断圆环中心是否在色块范围内
            color_ring_centers = []
            for ring_coord in ring_coords:
                ring_x, ring_y, ring_r = ring_coord
                if(int(x_min) < int(ring_x) < int(x_max) and int(y_min) < int(ring_y) < int(y_max)):
                    found_flag = 1
                    color_ring_centers.append((ring_x/640, ring_y/480))

                    cv2.circle(frame, (int(ring_x), int(ring_y)), int(ring_r), (0, 255, 0), 1)
                    cross_length = 5
                    cross_thickness = 1
                    cross_color = (0, 255, 0)
                    
                    # 水平线
                    cv2.line(frame, (int(ring_x) - cross_length, int(ring_y)), (int(ring_x) + cross_length, int(ring_y)), cross_color, cross_thickness) 
                    
                    # 垂直线
                    cv2.line(frame, (int(ring_x), int(ring_y) - cross_length), (int(ring_x), int(ring_y) + cross_length), cross_color, cross_thickness)
                    

            cv2.imshow("Color Ring", frame)
            cv2.waitKey(1)                            

            
        if not color_ring_centers:
            return None, 0

        return color_ring_centers[0], found_flag 
        
        

    def get_closer_to_car_color_ring(self, duration=0.5, still_threshold=2, MIN_AREA=15000, MAX_AREA=39000, r_min=70, r_max=100, smooth_factor=0.4):
        """
        获取离车最近（屏幕上方）的色块颜色及其圆环中心坐标
        
        参数:
        duration (float): 采集图像帧的持续时间，默认值为0.5秒
        still_threshold (int): 判断物块静止的阈值，默认值为2
        MIN_AREA (int): 色块面积的最小值，默认值为15000
        MAX_AREA (int): 色块面积的最大值，默认值为39000
        r_min (int): 圆环内径的最小半径
        r_max (int): 圆环外径的最大半径
        smooth_factor (float): 时间滤波的平滑因子
        
        返回:
        color_id (int): 颜色ID（1=红色，2=绿色，3=蓝色），未检测到则为None
        ring_center (tuple): 圆环中心坐标(x,y,r)，未检测到则为None
        found_flag (bool): 是否检测到色块
        still_flag (bool): 色块是否静止
        """
        # 获取离车最近的色块颜色和坐标
        color_id, target_coords, found_flag, still_flag = self.get_closer_to_car_center(
            duration=duration, 
            still_threshold=still_threshold, 
            MIN_AREA=MIN_AREA, 
            MAX_AREA=MAX_AREA,
            Mode="single"
        )
        
        # 如果没有检测到色块或色块不静止，直接返回
        if not found_flag:
            print("未检测到色块")
            return None, None, found_flag, still_flag
        
        if not still_flag:
            print("物料在动，无法精确定位圆环")
            return color_id, None, found_flag, still_flag
        
        if target_coords is None or len(target_coords) < 2:
            print("目标坐标无效")
            return color_id, None, found_flag, still_flag
        
        # 获取当前帧
        ret, frame = self.cap.read()
        if not ret or frame is None:
            print("无法获取图像帧")
            return color_id, None, found_flag, still_flag
        
        # 检测圆环
        ring_coords = self.get_ring_center(frame, r_min, r_max, smooth_factor)
        
        if not ring_coords:
            print("未检测到圆环")
            return color_id, None, found_flag, still_flag
        
        # 判断圆环中心是否在色块范围内
        color_ring_centers = []
        for ring_coord in ring_coords:
            ring_x, ring_y, ring_r = ring_coord
            
            # 计算圆环中心与色块中心的距离
            color_x = target_coords[0]
            color_y = target_coords[1]
            distance = np.sqrt((ring_x - color_x*640)**2 + (ring_y - color_y*480)**2)
            
            # 如果距离小于一定阈值，认为圆环中心在色块范围内
            if distance < ring_r * 1.5:  # 使用圆环半径的1.5倍作为阈值
                color_ring_centers.append((ring_x, ring_y, ring_r))
                break
        
        # 显示结果
        if color_ring_centers:
            # 在图像上标记检测到的圆环
            for x, y, r in color_ring_centers:
                cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 1)
                
                # 绘制十字线
                cross_length = 5
                cross_thickness = 1
                cross_color = (0, 255, 0)
                
                # 水平线
                cv2.line(frame, (int(x) - cross_length, int(y)), 
                        (int(x) + cross_length, int(y)), 
                        cross_color, cross_thickness) 
                
                # 垂直线
                cv2.line(frame, (int(x), int(y) - cross_length), 
                        (int(x), int(y) + cross_length), 
                        cross_color, cross_thickness)
                
                # 显示颜色ID
                cv2.putText(frame, f"Color: {color_id}", (30, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # 显示静止状态
                still_text = "Still: Yes" if still_flag else "Still: No"
                cv2.putText(frame, still_text, (400, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow("Closest Color Ring", frame)
            cv2.waitKey(1)
            
            # 返回第一个检测到的圆环中心
            return color_id, color_ring_centers[0], found_flag, still_flag
        
        # 如果没有找到匹配的圆环，返回颜色ID但圆环中心为None
        return color_id, None, found_flag, still_flag