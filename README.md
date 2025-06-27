# Yolov8PredectnumsAndControl
## 🧩 项目简介

本项目基于 YOLOv8 模型，结合树莓派5 + PiCamera2，实现对画面中排列的十六进制字符（0~F）的实时识别与追踪，并控制舵机通过激光点进行标定或定位。支持本地训练、ONNX部署与激光伺服控制功能。

------

## 📦 1. 软件运行环境

### 📍 主机训练环境

- 操作系统：Windows 10/11

- Python 版本：`>=3.8`

- YOLO 框架：`Ultralytics YOLOv8`

- 虚拟环境建议：`conda` 或 `venv`

- 关键依赖：

  ```
  bash
  
  
  复制编辑
  pip install ultralytics opencv-python numpy matplotlib
  ```

### 📍 树莓派部署环境（运行模型识别 + 激光控制）

- 系统版本：`Raspberry Pi OS Bookworm (Debian 12)`

- 树莓派型号：`树莓派 5`

- Python：`3.11`

- 虚拟环境：推荐使用 `venv`，如 `yolov8`

- 安装依赖：

  ```
  bash复制编辑pip install numpy opencv-python onnxruntime
  sudo apt install python3-lgpio python3-picamera2 pigpio
  ```

- 摄像头支持库：`Picamera2`

- 舵机控制库：

  - 推荐：`lgpio`（替代抖动严重的 RPi.GPIO）
  - 替代：`RPi.GPIO`（易用性高但稳定性差）

- 模型推理引擎：`onnxruntime`

------

## 🧱 2. 项目硬件环境

| 硬件         | 型号/说明                                  |
| ------------ | ------------------------------------------ |
| 主控板       | 树莓派 5                                   |
| 摄像头       | 官方 PiCamera2（兼容 OV5647/IMX219 等）    |
| 舵机         | MG996R 或 SG90（PWM 控制）                 |
| 舵机控制方式 | GPIO + PWM（推荐使用 `lgpio.tx_pwm` 控制） |
| 激光模块     | 普通红色激光笔（GPIO 控制开关）            |
| 电源         | PD 5V 3A 电源或移动电源                    |



------

## 🧠 3. 模型训练说明

### 数据集结构（以 `data.yaml` 为例）：

```
yaml复制编辑path: C:\Users\Buer_vakabauta\Desktop\predect\dataset
train: train/images
val: val/images
nc: 16
names:
- '0' - '1' - '2' - '3' - '4' - '5' - '6' - '7'
- '8' - '9' - 'A' - 'B' - 'C' - 'D' - 'E' - 'F'
```

### 训练命令

```
bash复制编辑yolo detect train \
  model=yolov8n.pt \
  data=data.yaml \
  epochs=100 \
  imgsz=640 \
  batch=16 \
  name=hex_classifier
```

### 导出 ONNX 模型

```
bash


复制编辑
yolo export model=runs/detect/hex_classifier/weights/best.pt format=onnx
```

------

## 📷 4. 摄像头模块（CameraCapture）

- 使用 Picamera2 获取图像流
- 独立线程采集图像并实时进行激光点检测（`laser_track`）
- 支持 `frame_queue` 异步访问，避免主线程阻塞

------

## 🤖 5. 舵机控制（Servo2）

- 使用 `lgpio` 实现高精度 PWM 控制，减少抖动
- 实现激光点“追踪”功能，根据激光位置微调角度
- 支持位置限幅，避免误差过大导致死循环

------

## 🧪 6. 推理与识别模块（YOLODetector）

- 使用 ONNX 模型部署在树莓派上
- 支持自定义置信度阈值、NMS 阈值
- 支持绘图可视化，显示检测框与分类结果
- 兼容 Ultralytics YOLOv8 的输出结构，适配 `onnxruntime`

------

## 🔄 7. 通信模块（serial_task.py）

- 支持与 STM32/下位机串口通信
- 后台线程接收串口消息
- 异常处理健壮性加强（防止 `bad file descriptor`）

------

## 🧯 8. 注意事项与常见问题

### ✅ 模型在 PC 能识别，在树莓派 ONNX 效果差？

- 检查 ONNX 输入是否使用了 `sigmoid(obj_conf)*cls_conf`
- 注意推理前的图像预处理需保持一致（padding, resize）

### ✅ 摄像头显示 `cv2.imshow()` 阻塞？

- 建议使用主线程处理 `cv2.imshow()`，子线程仅负责 `capture_array()`
- 或改用 `cv2.startWindowThread()` 等非阻塞方法

### ✅ `pigpio` 无法连接？

- 确保启动了 `sudo pigpiod`
- 或使用 `lgpio` 替代，兼容性更强

------

## 📂 9. 项目结构建议

```
bash复制编辑yolov8_laser_project/
├── main.py                # 主控制逻辑
├── camera.py              # 摄像头图像采集模块
├── servo.py               # 舵机控制模块（lgpio版）
├── serial_task.py         # 串口通信线程
├── yolodect.py            # ONNX模型推理
├── LaserTracker.py        # 激光点识别算法（基于颜色/亮度）
├── weights/
│   └── best.onnx          # 导出的ONNX模型
├── dataset/
│   ├── train/val/labels/images
│   └── data.yaml
```

------

## ✅ 10. 项目运行流程

```
bash


复制编辑
python main.py
```

主流程执行：

1. 初始化相机并采集激光初始位置
2. 自动或手动测距，计算 disx/disy
3. 进行字符检测识别
4. 激光点移动以靠近字符目标点（伺服控制）
5. 等待输入或串口指令，完成任务
