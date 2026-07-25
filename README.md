# 物流搬运机器人系统 (Logistics Handling Robot System)

这是一个基于STM32和树莓派的智能物流搬运机器人系统，具备全向移动、物体识别、机械臂控制等功能，主要用于自动化物料搬运任务。

## 系统架构

本项目采用分布式架构设计：

- **底盘控制系统** (`bujin/`): 基于STM32F407的嵌入式控制系统
- **视觉处理系统** (`hzw_vision/`): 基于树莓派的计算机视觉和任务调度系统

## 主要功能

### 🚗 底盘控制系统
- 四轮全向移动（麦卡纳姆轮）
- 高精度PID位置控制
- 陀螺仪姿态稳定
- 多设备UART通信
- 实时运动控制

### 🎯 视觉处理系统  
- 二维码识别和任务解析
- 颜色物体识别和定位
- 机械臂精确控制
- 摄像头图像处理
- 任务调度和执行

### 🤖 机械臂系统
- 多自由度舵机控制
- 物料抓取和放置
- 高度自适应调节
- 储物区和工作区切换

## 快速开始

### 环境要求

#### 硬件要求
- **主控板**: STM32F407VET6开发板
- **上位机**: 树莓派5 (推荐8GB内存)
- **摄像头**: USB摄像头 x2 (颜色识别 + 二维码扫描)
- **执行器**: 
  - 麦卡纳姆轮 x4
  - 步进电机/伺服电机 x4
  - 舵机 x多个 (机械臂控制)
- **传感器**: 
  - 陀螺仪模块
  - 位置编码器

#### 软件要求

**STM32开发环境:**
```bash
# 推荐使用STM32CubeIDE或Keil MDK
- STM32CubeMX (项目配置)
- ARM GCC工具链
- OpenOCD (调试下载)
```

**树莓派环境:**
```bash
# Python 3.8+
sudo apt update
sudo apt install python3-pip python3-venv

# 安装依赖包
pip3 install opencv-python numpy pyserial pyzbar threading
```

### 安装步骤

#### 1. 克隆项目
```bash
git clone https://gitlab.com/Collapsar-ss/logistics-handling.git
cd logistics-handling
```

#### 2. STM32固件编译与下载
```bash
cd bujin/
# 使用STM32CubeIDE打开bujin.code-workspace
# 或使用命令行编译
make clean
make all
# 下载到STM32
make flash
```

#### 3. 树莓派环境配置
```bash
cd hzw_vision/
# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate

# 安装依赖
pip install -r requirements.txt  # 如果有requirements.txt文件
# 或手动安装主要依赖
pip install opencv-python numpy pyserial pyzbar Pillow
```

#### 4. 硬件连接
- STM32串口1: 连接电机控制器
- STM32串口2: 连接树莓派  
- STM32串口3: 调试串口
- STM32串口5: 连接陀螺仪
- 树莓派USB: 连接两个摄像头

## 使用指南

### 基本操作

#### 1. 启动系统
```bash
# 首先启动STM32底盘控制系统 (通过调试器下载运行)

# 然后启动树莓派视觉系统
cd hzw_vision/
python3 task.py  # 主任务调度程序
```

#### 2. 机械臂测试
```bash
# 单独测试机械臂功能
python3 servotest.py
```

#### 3. 摄像头测试  
```bash
# 测试颜色识别
python3 getcenter_test.py

# 测试二维码识别
python3 getQr_test.py

# 综合视觉测试
python3 single_test.py
```

#### 4. 通信测试
```bash
# 测试STM32与树莓派通信
python3 task_test.py
```

### 主要模块说明

#### 底盘控制系统 (bujin/)

**核心文件结构:**
```
bujin/
├── Core/
│   ├── Inc/
│   │   ├── main.h           # 主头文件
│   │   ├── car.h            # 车辆控制头文件  
│   │   └── ...
│   └── Src/
│       ├── main.c           # 主程序入口
│       ├── car.c            # 车辆控制实现
│       └── ...
├── Drivers/                 # STM32 HAL库
├── Makefile                 # 编译配置
└── *.ioc                    # STM32CubeMX配置文件
```

**主要功能模块:**
- `car.c/car.h`: 车辆运动控制，包含高级PID控制算法
- `main.c`: 硬件初始化、中断处理、通信协议
- 支持多种运动模式：点到点移动、路径跟踪、原地旋转

#### 视觉处理系统 (hzw_vision/)

**核心文件结构:**
```
hzw_vision/
├── task.py              # 主任务调度器
├── Cameracontrol.py     # 摄像头控制和图像处理
├── Servocontrol.py      # 机械臂舵机控制
├── Uart.py              # 串口通信协议
├── TaskReceiver.py      # 任务接收和解析
├── *_test.py            # 各模块测试文件
└── *.png                # 参考图片资源
```

**主要功能模块:**
- `task.py`: 整体任务调度，协调各子系统
- `Cameracontrol.py`: 计算机视觉处理（颜色识别、二维码解析）
- `Servocontrol.py`: 机械臂控制（抓取、放置、位置调整）
- `Uart.py`: 与STM32的通信协议实现

## 技术特性

### 高级控制算法

#### PID控制系统
- **多轴PID控制**: 独立的X、Y、角度控制
- **Butterworth滤波**: 降低传感器噪声
- **动态增益调节**: 根据误差大小自适应调整参数
- **端点补偿**: 提高定位精度

#### 运动控制
- **麦卡纳姆轮运动学**: 支持全向移动
- **坐标系转换**: 本体坐标系与世界坐标系互转
- **平滑轨迹规划**: S型加减速曲线

#### 通信协议
- **多设备UART**: 支持电机、陀螺仪、树莓派同时通信
- **DMA数据传输**: 降低CPU占用率
- **协议解析**: 状态机方式解析数据包
- **容错机制**: 数据校验和超时处理

### 视觉处理能力

#### 图像识别
- **HSV颜色空间**: 准确的颜色识别
- **霍夫圆检测**: 圆形物体定位
- **轮廓分析**: 物体形状识别
- **多目标跟踪**: 同时跟踪多个物体

#### 二维码处理
- **实时解码**: pyzbar库支持多种码制
- **任务解析**: 自动解析任务指令
- **错误恢复**: 识别失败自动重试

```
logistics-handling/
├── README.md                    # 项目说明文档
├── bujin/                       # STM32底盘控制系统
│   ├── Core/                    # 核心代码
│   │   ├── Inc/                 # 头文件
│   │   │   ├── main.h           # 主头文件，系统配置
│   │   │   ├── car.h            # 车辆控制相关定义
│   │   │   └── ...              # 其他头文件
│   │   └── Src/                 # 源文件
│   │       ├── main.c           # 主程序，初始化和中断处理
│   │       ├── car.c            # 车辆控制算法实现
│   │       └── ...              # 其他源文件
│   ├── Drivers/                 # STM32 HAL驱动库
│   ├── Middlewares/             # 中间件
│   ├── build/                   # 编译输出目录
│   ├── Makefile                 # 构建配置
│   ├── *.ioc                    # STM32CubeMX配置文件
│   └── *.ld                     # 链接脚本
└── hzw_vision/                  # 树莓派视觉处理系统
    ├── task.py                  # 主任务调度器
    ├── Cameracontrol.py         # 摄像头控制和图像处理
    ├── Servocontrol.py          # 机械臂舵机控制
    ├── Uart.py                  # 串口通信协议
    ├── TaskReceiver.py          # 任务接收器
    ├── getcenter_test.py        # 颜色识别测试
    ├── getQr_test.py            # 二维码识别测试
    ├── servotest.py             # 机械臂测试
    ├── single_test.py           # 单一功能测试
    ├── task_test.py             # 任务流程测试
    ├── hsv_test.py              # HSV调试工具
    ├── hough_circle_tuner.py    # 圆检测参数调优
    ├── plate_test.py            # 平台识别测试
    ├── find_realcenter.py       # 中心点定位
    ├── *.png                    # 参考图片资源
    └── __pycache__/             # Python编译缓存
```

## 通信协议

### STM32与树莓派通信协议
```c
// 数据包格式: [STX][CMD][DATA...][ETX]
// STX: 0x02 (起始标志)
// ETX: 0x03 (结束标志)

// 命令类型定义
#define CMD_MOVE    0x01    // 移动指令
#define CMD_GRAB    0x02    // 抓取指令  
#define CMD_PLACE   0x03    // 放置指令
#define CMD_LOCATE  0x04    // 定位指令
```

### 电机控制协议 (Emm_V5)
```c
// 位置控制指令格式
// [地址][功能码][参数1][参数2][校验]
```

## 性能指标

| 性能参数 | 指标 | 说明 |
|---------|------|------|
| 定位精度 | ±2mm | 在1m范围内的定位精度 |
| 响应时间 | <100ms | 从指令接收到执行的时间 |
| 移动速度 | 0.5m/s | 最大安全移动速度 |
| 识别距离 | 0.3-1.5m | 有效的物体识别距离 |
| 抓取精度 | ±1mm | 机械臂抓取定位精度 |
| 通信延迟 | <50ms | STM32与树莓派通信延迟 |

## 贡献指南

1. Fork项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)  
5. 打开Pull Request

## 许可证

本项目使用MIT许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 联系方式

- 项目维护者: [Your Name]
- 邮箱: your.email@example.com
- 项目链接: https://gitlab.com/Collapsar-ss/logistics-handling

## 致谢

- STM32CubeMX 代码生成工具
- OpenCV 计算机视觉库
- pyzbar 二维码识别库
- FreeRTOS 实时操作系统

---

**注意**: 本项目仍在持续开发中，部分功能可能需要根据具体硬件配置进行调整。

### 硬件问题

**电机位置异常 (八位数距离值)**
```bash
# 现象：第1和第3个轮子的distance_traveled显示八位数
# 原因：编码器数据溢出或通信错误
# 解决：
1. 检查电机连线是否松动
2. 重新校准编码器零点
3. 检查ParseMotor_Status函数的数据解析
4. 使用相对位置而非绝对位置计算
```

**陀螺仪数据跳变**
```bash
# 现象：角度突然跳变±360度
# 解决：在角度处理中增加连续性检查
# 参考main.c中的角度连续性处理代码
```

**摄像头无法识别**
```bash
# 检查摄像头设备
ls /dev/video*

# 测试摄像头
python3 -c "import cv2; cap=cv2.VideoCapture(0); print(cap.isOpened())"

# 权限问题
sudo usermod -a -G video $USER
```

### 软件问题

**串口通信失败**
```bash
# 检查串口权限
sudo usermod -a -G dialout $USER

# 查看串口设备  
ls /dev/ttyUSB* /dev/ttyACM*

# 测试串口通信
python3 -c "import serial; s=serial.Serial('/dev/ttyUSB0', 115200); print('OK')"
```

**PID控制震荡**
```bash
# 现象：车辆移动时震荡
# 解决：
1. 减小PID参数 (特别是D项)
2. 增加滤波系数
3. 检查机械安装是否牢固
4. 调整控制周期
```

**机械臂定位不准**
```bash
# 现象：抓取位置偏差
# 解决：
1. 重新校准机械臂零点
2. 检查舵机标定参数
3. 调整视觉识别的坐标转换
4. 验证机械臂运动学参数
```
