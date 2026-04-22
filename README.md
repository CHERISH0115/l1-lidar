# L1 LiDAR Visualizer

基于 **宇树 L1 激光雷达 + 树莓派 + ROS2 Jazzy + Qt5** 的点云采集与实时可视化系统。
通过 Docker 容器化部署，一键启动，无需手动配置 ROS2 环境。

---

## 系统架构

```
宇树 L1 雷达 ──/dev/ttyUSB0──> l1_driver (ROS2节点)
                                      │ /scan (PointCloud2)
                                      ▼
                              qt_visualizer (Qt5 GUI)
                              3D 点云实时渲染 + PCD导出
```

---

## 目录结构

```
code/
├── Dockerfile.lidar        # 自定义 Docker 镜像（含 Qt5 + ROS2 Jazzy）
├── start_lidar.sh          # 一键启动脚本
├── l1_driver/              # ROS2 宇树L1雷达驱动包
│   ├── include/l1_driver/
│   │   └── l1_parser.hpp       # 串口协议解析（MAVLink v2）
│   ├── src/
│   │   ├── l1_parser.cpp
│   │   └── l1_driver_node.cpp  # 串口读取 → /scan 发布
│   └── launch/
│       └── l1_driver.launch.py
├── qt_visualizer/          # C++/Qt5 点云可视化界面
│   ├── include/
│   │   ├── cloud_widget.hpp    # 3D 点云渲染控件（轨道相机）
│   │   ├── ros_worker.hpp      # ROS2 后台订阅线程
│   │   └── main_window.hpp     # 主窗口
│   └── src/
│       ├── main.cpp
│       ├── main_window.cpp
│       ├── cloud_widget.cpp
│       └── ros_worker.cpp
├── stm32_remote/           # STM32 遥控终端（HAL库）
│   └── Core/
│       ├── Inc/  (lora.h / oled.h / key.h)
│       └── Src/  (main.c / lora.c / oled.c / key.c)
└── lora_bridge/            # 树莓派端 LoRa 桥接 ROS2 节点
    └── src/lora_bridge_node.cpp
```

---

## 快速启动（已配置好的设备）

```bash
cd ~/WYF/code
bash start_lidar.sh
```

脚本自动完成：构建 Docker 镜像 → 编译 ROS2 包 → 启动驱动和 Qt 界面。

---

## 移植到新设备（详细步骤）

### 前提条件

| 项目 | 要求 |
|------|------|
| 硬件 | 树莓派 4B / 5，arm64 架构 |
| 系统 | Ubuntu 24.04 Noble 64-bit（推荐）|
| 桌面 | Wayland 会话（Ubuntu 24.04 默认）|
| 雷达 | 宇树 L1，USB 连接后识别为 `/dev/ttyUSB0` |

---

### 第一步：安装 Docker

```bash
# 一键安装
curl -fsSL https://get.docker.com | sh

# 将当前用户加入 docker 组（避免每次 sudo）
sudo usermod -aG docker $USER

# 使组变更立即生效
newgrp docker

# 验证安装成功
docker run --rm hello-world
```

> 如果 `curl` 下载慢，可以用国内镜像：
> ```bash
> curl -fsSL https://mirror.nju.edu.cn/docker-ce/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker.gpg
> echo "deb [arch=arm64 signed-by=/usr/share/keyrings/docker.gpg] https://mirror.nju.edu.cn/docker-ce/linux/ubuntu noble stable" | sudo tee /etc/apt/sources.list.d/docker.list
> sudo apt update && sudo apt install -y docker-ce docker-ce-cli
> sudo usermod -aG docker $USER && newgrp docker
> ```

---

### 第二步：安装 Git 并克隆项目

```bash
# 安装 git（Ubuntu 通常已预装）
sudo apt install -y git

# 克隆项目
git clone https://github.com/CHERISH0115/l1-lidar.git

# 进入项目目录
cd l1-lidar
```

---

### 第三步：配置串口权限

```bash
# 插上雷达，确认设备已识别
ls /dev/ttyUSB*
# 应看到 /dev/ttyUSB0

# 将当前用户加入 dialout 组
sudo usermod -aG dialout $USER
newgrp dialout

# 验证权限
ls -l /dev/ttyUSB0
# 应显示 crw-rw---- ... dialout ...
```

---

### 第四步：确认 Wayland 环境

Qt 界面通过 Wayland 显示，Ubuntu 24.04 默认已是 Wayland 会话。

```bash
# 验证当前会话类型
echo $XDG_SESSION_TYPE
# 输出应为 wayland
```

如果输出 `x11`，请注销后在登录界面右下角选择 **Ubuntu（Wayland）** 重新登录。

---

### 第五步：一键启动

```bash
bash start_lidar.sh
```

**首次运行耗时说明：**

| 步骤 | 说明 | 耗时 |
|------|------|------|
| 构建 Docker 镜像 | 安装 Qt5 + ROS2 依赖 | 约 3 分钟 |
| 编译 l1_driver | ROS2 串口驱动包 | 约 30 秒 |
| 编译 qt_visualizer | Qt5 点云界面 | 约 2 分钟 |

**后续运行**直接秒启，无需重新编译。

---

### 第六步：使用 Qt 界面

Qt 窗口启动后即开始实时显示点云：

| 操作 | 功能 |
|------|------|
| 鼠标左键拖拽 | 旋转 3D 视角 |
| 滚轮 | 缩放 |
| Frames 数值框 | 调整点云密度（帧缓冲数，默认200帧）|
| **Clear Map** | 清除当前点云 |
| **Reset View** | 恢复默认视角 |
| **Export PCD** | 导出当前点云为 `.pcd` 文件 |

导出的 `.pcd` 文件可用 [CloudCompare](https://www.cloudcompare.org/) 打开进一步处理。

---

## 常见问题

### Qt 窗口不显示

```bash
# 确认 Wayland 会话
echo $XDG_SESSION_TYPE   # 应为 wayland

# 确认 wayland socket 存在
ls /run/user/$(id -u)/wayland-0
```

### 找不到 /dev/ttyUSB0

```bash
# 检查内核是否识别到设备
dmesg | grep tty | tail -10

# 临时授权（重启后失效）
sudo chmod 666 /dev/ttyUSB0
```

### 点云界面无数据

```bash
# 进入容器检查驱动是否正常发布
docker exec -it l1_driver bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 topic hz /scan"
# 正常应显示 10 Hz 左右的频率
```

### 重新编译（代码更新后）

```bash
rm -rf build/ install/ log/
bash start_lidar.sh
```

### Docker 镜像需要重建

```bash
docker rmi lidar-ros:latest
bash start_lidar.sh   # 会自动重新构建
```

---

## 话题说明

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | `sensor_msgs/PointCloud2` | 雷达原始点云（主话题）|
| `/imu/data_raw` | `sensor_msgs/Imu` | IMU 原始数据 |
| `/Odometry` | `nav_msgs/Odometry` | 里程计输出 |

---

## 硬件连接

```
宇树 L1  ──USB 串口──> 树莓派 /dev/ttyUSB0
LoRa 模块 ──UART────> 树莓派 /dev/ttyS0
LoRa 模块 ──UART────> STM32 USART2 (PA2/PA3, 9600bps)
OLED      ──I2C─────> STM32 I2C1   (400kHz)
按键 K1~K4 ──GPIO──> STM32 PA0/PA1/PA4/PA5（输入上拉）
```

### STM32 编译（Keil / CubeIDE）

1. STM32CubeMX 新建工程，芯片选 **STM32F103C8T6**
2. 配置外设：USART2（PA2/PA3，9600）、I2C1（400kHz）、GPIO 输入上拉
3. 将 `stm32_remote/Core/Src/*.c` 和 `Core/Inc/*.h` 复制到工程目录
4. 编译并烧录
