# 代码目录结构与编译说明

## 目录结构

```
code/
├── l1_driver/          # ROS2 宇树L1激光雷达驱动包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/l1_driver/
│   │   └── l1_parser.hpp       # 串口协议解析类
│   ├── src/
│   │   ├── l1_parser.cpp       # 串口解析实现
│   │   └── l1_driver_node.cpp  # ROS2驱动节点
│   └── launch/
│       └── l1_driver.launch.py
│
├── qt_visualizer/      # C++/Qt 轻量化可视化界面
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── cloud_widget.hpp    # 点云渲染控件
│   │   ├── ros_worker.hpp      # ROS2线程工作器
│   │   └── main_window.hpp     # 主窗口
│   └── src/
│       ├── main.cpp
│       ├── main_window.cpp
│       ├── cloud_widget.cpp
│       └── ros_worker.cpp
│
├── stm32_remote/       # STM32遥控终端（HAL库）
│   └── Core/
│       ├── Inc/
│       │   ├── lora.h          # LoRa通信头文件
│       │   ├── oled.h          # OLED显示头文件
│       │   └── key.h           # 按键扫描头文件
│       └── Src/
│           ├── main.c          # 主程序
│           ├── lora.c          # LoRa收发实现
│           ├── oled.c          # SSD1306驱动
│           └── key.c           # 按键消抖扫描
│
└── lora_bridge/        # 树莓派端LoRa桥接ROS2节点
    ├── CMakeLists.txt
    ├── package.xml
    ├── src/
    │   └── lora_bridge_node.cpp
    └── launch/
        └── lora_bridge.launch.py
```

## 编译与运行

### ROS2 包（树莓派）

```bash
# 进入 ROS2 工作空间
cd ~/ros2_ws/src
cp -r /path/to/code/l1_driver   .
cp -r /path/to/code/qt_visualizer .
cp -r /path/to/code/lora_bridge .

cd ~/ros2_ws
colcon build --packages-select l1_driver lora_bridge qt_visualizer
source install/setup.bash

# 启动雷达驱动
ros2 launch l1_driver l1_driver.launch.py serial_port:=/dev/ttyUSB0

# 启动 LoRa 桥接节点
ros2 launch lora_bridge lora_bridge.launch.py serial_port:=/dev/ttyS0

# 启动 Qt 可视化界面
ros2 run qt_visualizer qt_visualizer_node
```

### STM32（Keil / CubeIDE）

1. 用 STM32CubeMX 创建工程，芯片选 STM32F103C8T6
2. 配置外设：
   - USART2: 波特率9600, TX=PA2, RX=PA3 → LoRa
   - I2C1:  标准模式400kHz → OLED
   - GPIO:  PA0/PA1/PA4/PA5 输入上拉 → 按键
3. 将 `Core/Src/*.c` 和 `Core/Inc/*.h` 复制到工程对应目录
4. 编译烧录

## 话题说明

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | `sensor_msgs/PointCloud2` | 雷达点云 |
| `/imu/data_raw` | `sensor_msgs/Imu` | IMU数据 |
| `/Odometry` | `nav_msgs/Odometry` | Point-LIO里程计输出 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Point-LIO配准点云 |
| `/slam/start` | `std_msgs/Bool` | 启动建图指令 |
| `/slam/stop` | `std_msgs/Bool` | 停止建图指令 |

## 硬件连接

```
宇树L1 ──USB/串口──→ 树莓派 USB0
LoRa模块 ──UART──→ 树莓派 ttyS0
OLED ──I2C──→ STM32 I2C1
LoRa模块 ──UART──→ STM32 USART2
按键K1~K4 ──GPIO──→ STM32 PA0/PA1/PA4/PA5
```
