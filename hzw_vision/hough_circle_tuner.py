import cv2
import numpy as np
import time
from typing import Tuple, Optional

class CircleDetector:
    def __init__(self):
        # 颜色阈值
        self.color_thresholds = {
            1: {  # 红色
                'lower1': np.array([0, 60, 60]),
                'upper1': np.array([13, 255, 255]),
                'lower2': np.array([160, 50, 50]),
                'upper2': np.array([180, 255, 255])
            },
            2: {  # 绿色
                'lower': np.array([39, 40, 55]),
                'upper': np.array([95, 255, 250])
            },
            3: {  # 蓝色
                'lower': np.array([100, 60, 60]),
                'upper': np.array([124, 255, 255])
            }
        }
        
        # 霍夫圆检测参数
        self.params = {
            'dp': 0.5,  # 累加器分辨率
            'minDist': 50,
            'param1': 110,
            'param2': 80,
            'minRadius': 90,
            'maxRadius': 120,
            'alpha': 0.3  # 平滑系数
        }
        
        # 平滑滤波参数
        self.prev_x = None
        self.prev_y = None
        self.prev_r = None
        
        # 创建窗口和轨迹栏
        cv2.namedWindow('Tuner')
        self._create_trackbars()

    def _create_trackbars(self):
        """创建参数调节滑动条"""
        # 使用整数值乘以100来表示浮点数
        cv2.createTrackbar('dp*100', 'Tuner', int(self.params['dp']*100), 200, self._update_dp)
        cv2.createTrackbar('minDist', 'Tuner', self.params['minDist'], 200, self._update_minDist)
        cv2.createTrackbar('param1', 'Tuner', self.params['param1'], 300, self._update_param1)
        cv2.createTrackbar('param2', 'Tuner', self.params['param2'], 200, self._update_param2)
        cv2.createTrackbar('minRadius', 'Tuner', self.params['minRadius'], 200, self._update_minRadius)
        cv2.createTrackbar('maxRadius', 'Tuner', self.params['maxRadius'], 300, self._update_maxRadius)
        cv2.createTrackbar('alpha*100', 'Tuner', int(self.params['alpha']*100), 100, self._update_alpha)

    # 为每个参数创建单独的回调函数
    def _update_dp(self, value):
        self.params['dp'] = max(0.1, value / 100.0)
        
    def _update_minDist(self, value):
        self.params['minDist'] = value
        
    def _update_param1(self, value):
        self.params['param1'] = value
        
    def _update_param2(self, value):
        self.params['param2'] = value
        
    def _update_minRadius(self, value):
        self.params['minRadius'] = value
        
    def _update_maxRadius(self, value):
        self.params['maxRadius'] = value
        
    def _update_alpha(self, value):
        self.params['alpha'] = value / 100.0
        
    def preprocess_frame(self, frame: np.ndarray, color_number: int) -> Tuple[np.ndarray, np.ndarray]:
        """预处理图像帧"""
        # 转换到HSV空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 根据颜色创建掩码
        if color_number == 1:  # 红色需要两个范围
            mask1 = cv2.inRange(hsv, self.color_thresholds[1]['lower1'], self.color_thresholds[1]['upper1'])
            mask2 = cv2.inRange(hsv, self.color_thresholds[1]['lower2'], self.color_thresholds[1]['upper2'])
            mask = cv2.add(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, self.color_thresholds[color_number]['lower'], 
                             self.color_thresholds[color_number]['upper'])
        
        # 应用掩码
        color_filtered = cv2.bitwise_and(frame, frame, mask=mask)
        
        # 增强对比度和亮度
        lab = cv2.cvtColor(color_filtered, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        enhanced = cv2.merge((cl,a,b))
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        # enhanced = cv2.medianBlur(enhanced, 5)
    
        
        return color_filtered, enhanced

    def detect_circles(self, frame: np.ndarray, color_number: int) -> Tuple[Optional[float], Optional[float], np.ndarray, bool]:
        """检测圆形"""
        height, width = frame.shape[:2]
        
        # 缩小图像以提高处理速度
        scale = 0.5
        small_frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)
        
        # 颜色分割获取区域范围
        hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)
        if color_number == 1:  # 红色需要两个范围
            mask1 = cv2.inRange(hsv, self.color_thresholds[1]['lower1'], self.color_thresholds[1]['upper1'])
            mask2 = cv2.inRange(hsv, self.color_thresholds[1]['lower2'], self.color_thresholds[1]['upper2'])
            mask = cv2.add(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, self.color_thresholds[color_number]['lower'], 
                             self.color_thresholds[color_number]['upper'])
        
        # 找到颜色区域的边界
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        color_regions = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 10000 * scale * scale:  # 调整面积阈值以适应缩放
                x, y, w, h = cv2.boundingRect(cnt)
                color_regions.append((int(x/scale), int(y/scale), int((x+w)/scale), int((y+h)/scale)))
                cv2.rectangle(frame, (int(x/scale), int(y/scale)), 
                             (int((x+w)/scale), int((y+h)/scale)), (0, 0, 255), 2)
        
        # 如果没有找到颜色区域，提前返回
        if not color_regions:
            cv2.imshow('Mask', mask)
            return None, None, frame, False
        
        # 在原图上进行圆检测的预处理
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 使用更快的模糊方法
        blurred = cv2.medianBlur(gray, 5)
        
        # 使用更高效的边缘检测参数
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)
        
        # 霍夫圆检测
        circles = cv2.HoughCircles(
            edges,
            cv2.HOUGH_GRADIENT,
            dp=self.params['dp'],
            minDist=self.params['minDist'],
            param1=self.params['param1'],
            param2=self.params['param2'],
            minRadius=self.params['minRadius'],
            maxRadius=self.params['maxRadius']
        )
        
        # 显示处理过程
        cv2.imshow('Mask', mask)
        
        if circles is not None and len(color_regions) > 0:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                x, y, r = i[0], i[1], i[2]
                
                # 检查圆心是否在任何一个颜色区域内
                for x_min, y_min, x_max, y_max in color_regions:
                    if x_min < x < x_max and y_min < y < y_max:
                        # 应用平滑滤波
                        if self.prev_x is not None:
                            x = int(self.params['alpha'] * self.prev_x + (1 - self.params['alpha']) * x)
                            y = int(self.params['alpha'] * self.prev_y + (1 - self.params['alpha']) * y)
                            r = int(self.params['alpha'] * self.prev_r + (1 - self.params['alpha']) * r)
                        
                        # 更新历史位置
                        self.prev_x, self.prev_y, self.prev_r = x, y, r
                        
                        # 绘制圆和圆心
                        cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                        cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
                        
                        return x/width, y/height, frame, True
                
        return None, None, frame, False

def main():
    cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    # 设置较低的分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # 设置缓冲区大小为1，减少延迟
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    detector = CircleDetector()
    color_number = 1  # 默认检测红色
    
    prev_time = time.time()
    fps_values = []
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # 检测圆
        x_out, y_out, processed_frame, found = detector.detect_circles(frame, color_number)
        
        # 计算和显示FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        fps_values.append(fps)
        if len(fps_values) > 10:  # 保持最近10帧的平均值
            fps_values.pop(0)
        avg_fps = sum(fps_values) / len(fps_values)
        
        # 显示参数
        params_text = f"dp:{detector.params['dp']:.2f} minDist:{detector.params['minDist']} "
        params_text2 = f"param1:{detector.params['param1']} param2:{detector.params['param2']} "
        params_text3 = f"minR:{detector.params['minRadius']} maxR:{detector.params['maxRadius']} alpha:{detector.params['alpha']:.2f}"
        
        cv2.putText(processed_frame, f'FPS: {avg_fps:.1f}', (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(processed_frame, params_text, (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(processed_frame, params_text2, (10, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(processed_frame, params_text3, (10, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        if found:
            cv2.putText(processed_frame, f'Position: ({x_out:.2f}, {y_out:.2f})', 
                        (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('Frame', processed_frame)
        prev_time = curr_time
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            color_number = 1  # 红色
        elif key == ord('2'):
            color_number = 2  # 绿色
        elif key == ord('3'):
            color_number = 3  # 蓝色
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 