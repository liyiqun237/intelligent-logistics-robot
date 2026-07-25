# Intelligent Logistics Handling Robot

An autonomous mobile robot system developed for the National Engineering Innovation Competition.  
It integrates **STM32F407** (Real-time Motion Control) and **Raspberry Pi 5** (Vision & Decision), achieving high-precision omnidirectional movement and object manipulation.

![Robot System](docs/system_architecture.png)

## 🏆 Key Achievements

- **Positioning Accuracy:** ±2mm (Odometry + Gyroscope Fusion)
- **Grasping Precision:** ±1mm (Sub-pixel Visual Servoing)
- **Response Time:** < 100ms (UART @ 115200 Baud)
- **Award:** Provincial Third Prize, Guangdong Engineering Innovation Competition

## 📐 System Architecture

The system adopts a distributed architecture:

- **Chassis Control (STM32F407):** Handles motor driving, PID control, sensor fusion (Encoders + MPU6050), and real-time execution.
- **Vision & Brain (Raspberry Pi 5):** Processes images, recognizes colors/QR codes, plans paths, and sends commands via UART.

```
          UART (115200)
[RPi 5]  <---------------->  [STM32F407]
(Vision)                      (Actuation)
  |                                |
Camera                        Motors/Sensors
```

## 🚀 Features

### Hardware

- **MCU:** STM32F407VET6
- **SBC:** Raspberry Pi 5
- **Locomotion:** Mecanum Wheels (Omnidirectional)
- **Sensors:** Encoders (x4), Gyroscope (MPU6050), USB Cameras (x2)
- **Manipulator:** Multi-DOF Arm with Servos

### Software & Algorithms

- **Motion Control:** Mecanum Inverse Kinematics, Incremental PID with **Butterworth Filtering** (4th Order), S-Curve Acceleration.
- **Computer Vision:** LAB-CLAHE Enhancement, HSV Thresholding, **Hough Circle Transform**, Sub-pixel Center Extraction.
- **Communication:** Custom UART Protocol with CRC Checksum and DMA support.
- **State Machine:** Robust task scheduling for warehouse logistics scenarios.

## 🛠️ Getting Started

### Prerequisites

- **STM32 Environment:** STM32CubeIDE / Keil MDK, ARM GCC Toolchain.
- **Raspberry Pi Environment:** Python 3.8+, OpenCV, PySerial, PyZBar.

### Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/liyiqun237/intelligent-logistics-robot.git
   cd intelligent-logistics-robot
   ```

2. **Build Firmware (STM32):**  
   Navigate to `bujin/` and import into STM32CubeIDE, or use Makefile:

   ```bash
   cd bujin/
   make clean
   make all
   ```

3. **Setup Environment (Raspberry Pi):**

   ```bash
   cd hzw_vision/
   pip3 install opencv-python numpy pyserial pyzbar
   ```

## 📖 Usage

1. **Flash STM32:**  
   Download the compiled `.bin` file to the STM32 board via ST-Link using STM32CubeProgrammer or the IDE's built-in tools.

2. **Run Vision System:**

   ```bash
   cd hzw_vision/
   python3 task.py
   ```

3. **Testing Modules:**

   - Vision Test: `python3 getcenter_test.py`
   - QR Code Test: `python3 getQr_test.py`
   - Servo Test: `python3 servotest.py`

## 📁 Directory Structure

```
logistics-handling/
├── bujin/                      # STM32 Embedded Code
│   ├── Core/
│   │   ├── Src/                # C Source Files
│   │   │   ├── main.c          # Main loop, PID, Fusion
│   │   │   └── car.c           # Kinematics
│   │   └── Inc/                # Header Files
│   └── *.ioc                   # CubeMX Config
├── hzw_vision/                 # Raspberry Pi Python Code
│   ├── task.py                 # Main scheduler
│   ├── CameraControl.py        # OpenCV processing
│   ├── ServoControl.py         # Servo PWM control
│   └── Uart.py                 # Serial protocol
├── docs/                       # Documentation & Images
└── README.md
```

## ⚠️ Troubleshooting

- **Encoder Overflow:** If distance readings show abnormal large values, switch from absolute to relative position calculation in `car.c`.
- **Gyro Jump:** Implement angle continuity checking (±360° wrap-around) in the IMU data parsing routine.
- **PID Oscillation:** Reduce Derivative (D) gain or increase the cutoff frequency of the Butterworth filter.

## 📜 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

## 📧 Contact

Yiqun Li (HIT Shenzhen) — [GitHub Profile](https://github.com/liyiqun237)
