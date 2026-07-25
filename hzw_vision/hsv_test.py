import cv2
import numpy as np

class HSVTuner:
    def __init__(self):
        # 创建主窗口
        cv2.namedWindow('HSV Tuner')
        cv2.namedWindow('Color Masks')
        
        # 初始化HSV阈值
        self.hsv_values = {
            'red1': {'low': [0, 60, 60], 'high': [13, 255, 255]},
            'red2': {'low': [160, 50, 50], 'high': [180, 255, 255]},
            'green': {'low': [39, 39, 50], 'high': [95, 255, 255]},
            'blue': {'low': [100, 60, 60], 'high': [124, 255, 255]}
        }
        
        # 创建轨迹栏
        self._create_trackbars()
        
        # 当前选择的颜色
        self.current_color = 'red1'
        
    def _create_trackbars(self):
        """创建所有颜色的HSV调节滑动条"""
        def create_color_trackbars(color_name, window_name='HSV Tuner'):
            base_y = {'red1': 0, 'red2': 120, 'green': 240, 'blue': 360}
            
            # 创建颜色标签
            cv2.putText(self.label_img, f"{color_name.upper()}", 
                       (10, base_y[color_name] + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 低阈值
            cv2.createTrackbar(f'{color_name}_H_low', window_name, 
                             self.hsv_values[color_name]['low'][0], 180, 
                             lambda x: self._update_value(color_name, 'low', 0, x))
            cv2.createTrackbar(f'{color_name}_S_low', window_name,
                             self.hsv_values[color_name]['low'][1], 255,
                             lambda x: self._update_value(color_name, 'low', 1, x))
            cv2.createTrackbar(f'{color_name}_V_low', window_name,
                             self.hsv_values[color_name]['low'][2], 255,
                             lambda x: self._update_value(color_name, 'low', 2, x))
            
            # 高阈值
            cv2.createTrackbar(f'{color_name}_H_high', window_name,
                             self.hsv_values[color_name]['high'][0], 180,
                             lambda x: self._update_value(color_name, 'high', 0, x))
            cv2.createTrackbar(f'{color_name}_S_high', window_name,
                             self.hsv_values[color_name]['high'][1], 255,
                             lambda x: self._update_value(color_name, 'high', 1, x))
            cv2.createTrackbar(f'{color_name}_V_high', window_name,
                             self.hsv_values[color_name]['high'][2], 255,
                             lambda x: self._update_value(color_name, 'high', 2, x))
        
        # 创建标签图像
        self.label_img = np.zeros((480, 300, 3), dtype=np.uint8)
        
        # 为每种颜色创建轨迹栏
        create_color_trackbars('red1')
        create_color_trackbars('red2')
        create_color_trackbars('green')
        create_color_trackbars('blue')
        
        # 显示标签图像
        cv2.imshow('HSV Tuner', self.label_img)
    
    def _update_value(self, color_name, bound, index, value):
        """更新HSV值"""
        self.hsv_values[color_name][bound][index] = value
    
    def process_frame(self, frame):
        """处理图像帧并显示结果"""
        # 转换到HSV空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 创建每种颜色的掩码
        masks = {}
        
        # 红色需要两个范围
        red_mask1 = cv2.inRange(hsv, 
                              np.array(self.hsv_values['red1']['low']),
                              np.array(self.hsv_values['red1']['high']))
        red_mask2 = cv2.inRange(hsv,
                              np.array(self.hsv_values['red2']['low']),
                              np.array(self.hsv_values['red2']['high']))
        masks['red'] = cv2.bitwise_or(red_mask1, red_mask2)
        
        # 绿色和蓝色
        masks['green'] = cv2.inRange(hsv,
                                   np.array(self.hsv_values['green']['low']),
                                   np.array(self.hsv_values['green']['high']))
        masks['blue'] = cv2.inRange(hsv,
                                  np.array(self.hsv_values['blue']['low']),
                                  np.array(self.hsv_values['blue']['high']))
        
        # 创建结果图像
        results = {}
        for color in masks:
            results[color] = cv2.bitwise_and(frame, frame, mask=masks[color])
        
        # 显示结果
        # 创建一个大的显示窗口，包含所有颜色的结果
        h, w = frame.shape[:2]
        display = np.zeros((h * 2, w * 2, 3), dtype=np.uint8)
        
        # 原始图像
        display[:h, :w] = frame
        # 红色结果
        display[:h, w:] = results['red']
        # 绿色结果
        display[h:, :w] = results['green']
        # 蓝色结果
        display[h:, w:] = results['blue']
        
        # 添加标签
        cv2.putText(display, 'Original', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(display, 'Red', (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(display, 'Green', (10, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(display, 'Blue', (w + 10, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        # 显示当前HSV值
        self._show_hsv_values(display)
        
        cv2.imshow('Color Masks', display)
    
    def _show_hsv_values(self, img):
        """在图像上显示当前HSV值"""
        h = img.shape[0] // 2
        w = img.shape[1] // 2
        y_offset = 60
        
        # 显示红色HSV值
        cv2.putText(img, f"Red1 HSV: {self.hsv_values['red1']['low']} - {self.hsv_values['red1']['high']}", 
                   (w + 10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f"Red2 HSV: {self.hsv_values['red2']['low']} - {self.hsv_values['red2']['high']}", 
                   (w + 10, y_offset + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 显示绿色HSV值
        cv2.putText(img, f"Green HSV: {self.hsv_values['green']['low']} - {self.hsv_values['green']['high']}", 
                   (10, h + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 显示蓝色HSV值
        cv2.putText(img, f"Blue HSV: {self.hsv_values['blue']['low']} - {self.hsv_values['blue']['high']}", 
                   (w + 10, h + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def save_values(self):
        """保存当前HSV值到文件"""
        with open('hsv_values.txt', 'w') as f:
            for color, values in self.hsv_values.items():
                f.write(f"{color}:\n")
                f.write(f"  low: {values['low']}\n")
                f.write(f"  high: {values['high']}\n")
        print("HSV values saved to hsv_values.txt")

def main():
    # 初始化摄像头
    cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
    tuner = HSVTuner()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 使用CLAHE增强图像
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        enhanced = cv2.merge((cl,a,b))
        frame = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)

        # 处理帧
        tuner.process_frame(frame)
        
        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            tuner.save_values()
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 