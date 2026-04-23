# L1 LiDAR SLAM Visualizer

基于 **宇树 L1 激光雷达 + 树莓派 + ROS2 Jazzy + FAST-LIO2 + Qt5** 的实时 LiDAR-IMU 紧耦合 SLAM 系统。
通过 Docker 容器化部署，一键启动，无需手动配置 ROS2 环境。

---

## 系统架构

```
宇树 L1 雷达 ──/dev/ttyUSB0──> l1_driver (ROS2节点)
                                    │
                     ┌──────────────┴──────────────┐
                     │ /scan (PointCloud2+time)     │ /imu/data_raw (Imu)
                     └──────────────┬──────────────┘
                                    ▼
                              FAST-LIO2 (SLAM)
                         IEKF + IKD-tree 建图
                         LiDAR-IMU 紧耦合融合
                                    │
                     ┌──────────────┴──────────────┐
                     │ /cloud_registered            │ /Odometry
                     │ (全局注册点云)               │ (位置+姿态)
                     └──────────────┬──────────────┘
                                    ▼
                            qt_visualizer (Qt5 GUI)
                        全局地图实时渲染 + 运动轨迹 + PCD导出
```

---

## 目录结构

```
code/
├── Dockerfile.lidar        # 自定义 Docker 镜像（Qt5 + ROS2 Jazzy + PCL）
├── start_lidar.sh          # 一键启动脚本
├── l1_driver/              # ROS2 宇树L1雷达驱动包
│   ├── include/l1_driver/
│   │   └── l1_parser.hpp       # 串口协议解析（MAVLink v2）
│   └── src/
│       ├── l1_parser.cpp
│       └── l1_driver_node.cpp  # 串口读取 → /scan + /imu/data_raw 发布
├── fast_lio/               # FAST-LIO2 SLAM 包（LiDAR-IMU 紧耦合）
│   ├── config/
│   │   └── l1_lidar.yaml       # L1 专用参数（IMU噪声、外参、IEKF配置）
│   ├── launch/
│   │   └── fast_lio.launch.py
│   ├── include/
│   │   ├── common_lib.h        # 状态向量、点类型定义
│   │   ├── so3_math.h          # SO(3) 李群数学
│   │   ├── IMU_Processing.hpp  # IMU积分、去畸变、协方差传播
│   │   ├── preprocess.h        # 点云预处理头文件
│   │   └── ikd-Tree/
│   │       └── ikd_Tree.h      # 增量式KD-tree
│   └── src/
│       ├── ikd_Tree.cpp        # IKD-tree 实现
│       ├── preprocess.cpp      # 标准PointCloud2适配
│       └── laserMapping.cpp    # SLAM主节点（IEKF + 点到面配准）
├── qt_visualizer/          # C++/Qt5 SLAM可视化界面
│   ├── include/
│   │   ├── cloud_widget.hpp    # 3D点云渲染（全局地图 + 轨迹）
│   │   ├── ros_worker.hpp      # ROS2后台订阅线程
│   │   └── main_window.hpp     # 主窗口
│   └── src/
│       ├── main.cpp
│       ├── main_window.cpp
│       ├── cloud_widget.cpp
│       └── ros_worker.cpp
├── lora_bridge/            # 树莓派端 LoRa 桥接 ROS2 节点
│   └── src/lora_bridge_node.cpp
└── stm32_remote/           # STM32 遥控终端（HAL库）
    └── Core/
        ├── Inc/  (lora.h / oled.h / key.h)
        └── Src/  (main.c / lora.c / oled.c / key.c)
```

---

## 快速启动（已配置好的设备）

```bash
cd ~/WYF/code
bash start_lidar.sh
```

脚本自动完成：检测 Dockerfile 变更并按需重建镜像 → 编译所有 ROS2 包（增量编译）→ 启动驱动 + SLAM + Qt 界面。

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
sudo apt install -y git
git clone https://github.com/CHERISH0115/l1-lidar.git
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

```bash
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
| 构建 Docker 镜像 | 安装 Qt5 + ROS2 + PCL 依赖 | 约 5 分钟 |
| 编译全部包 | l1_driver + fast_lio + qt_visualizer | 约 5 分钟 |

**后续运行**直接秒启，colcon 增量编译只重建有变化的包；Dockerfile 变更时自动重建镜像。

---

### 第六步：使用 Qt 界面

Qt 窗口启动后，等待 SLAM 初始化（约 2~3 秒静止时间用于 IMU 对齐重力），随后开始实时建图：

| 操作 | 功能 |
|------|------|
| 鼠标左键拖拽 | 旋转 3D 视角 |
| 滚轮 | 缩放 |
| **Show trajectory** 复选框 | 显示/隐藏运动轨迹（红线）|
| Frames 数值框 | 调整地图点数上限（缓冲帧数）|
| **Clear Map** | 清除当前地图和轨迹 |
| **Reset View** | 恢复默认视角 |
| **Export PCD** | 导出当前全局地图为 `.pcd` 文件 |

导出的 `.pcd` 文件可用 [CloudCompare](https://www.cloudcompare.org/) 打开进一步处理。

---

## FAST-LIO2 参数调整

编辑 `fast_lio/config/l1_lidar.yaml` 后重新运行 `bash start_lidar.sh`（colcon 自动增量编译）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `blind` | 0.3 m | 近距离盲区，过小会引入噪声 |
| `det_range` | 40.0 m | 最大探测距离 |
| `filter_size_corner` | 0.1 m | 扫描点降采样体素尺寸 |
| `filter_size_map` | 0.2 m | 地图体素尺寸，越小地图越密 |
| `max_iteration` | 4 | IEKF 最大迭代次数 |
| `imu_acc_n` | 0.1 | 加速度计噪声密度（标定后替换）|
| `imu_gyr_n` | 0.01 | 陀螺仪噪声密度（标定后替换）|
| `extrinsic_R/T` | 单位阵/零 | IMU ↔ LiDAR 外参（L1 内置 IMU 近似单位阵）|

---

## 常见问题

### Qt 窗口不显示

```bash
echo $XDG_SESSION_TYPE   # 应为 wayland
ls /run/user/$(id -u)/wayland-0
```

### 找不到 /dev/ttyUSB0

```bash
dmesg | grep tty | tail -10
sudo chmod 666 /dev/ttyUSB0   # 临时授权
```

### SLAM 不收敛 / 漂移严重

```bash
# 检查 IMU 数据是否正常
docker exec -it l1_driver bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 topic hz /imu/data_raw"
# 正常应为 ~200 Hz

# 检查点云是否正常
docker exec -it l1_driver bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 topic hz /scan"
# 正常应为 ~10 Hz
```

启动时保持传感器**静止 2~3 秒**，等待 IMU 重力方向初始化完成。

### 点云界面无数据（无建图输出）

```bash
# 确认 FAST-LIO2 正在发布里程计
docker exec -it l1_driver bash -c \
  "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 topic hz /Odometry"
# 正常应为 ~10 Hz
```

### 重新编译（代码更新后）

`start_lidar.sh` 每次运行都会自动增量编译，直接重跑脚本即可：

```bash
bash start_lidar.sh
```

如需完整重编译：

```bash
rm -rf build/ install/ log/
bash start_lidar.sh
```

### Docker 镜像需要重建

修改 `Dockerfile.lidar` 后，脚本会根据文件 SHA256 自动检测变更并重建镜像，无需手动操作。

如需强制重建：

```bash
docker rmi $(docker images lidar-ros -q)
bash start_lidar.sh
```

---

## 话题说明

| 话题 | 类型 | 发布节点 | 说明 |
|------|------|----------|------|
| `/scan` | `sensor_msgs/PointCloud2` | l1_driver | 原始点云（含每点时间戳 `time` 字段）|
| `/imu/data_raw` | `sensor_msgs/Imu` | l1_driver | IMU 原始数据（~200 Hz）|
| `/Odometry` | `nav_msgs/Odometry` | fast_lio | SLAM 位姿估计（世界坐标系）|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | fast_lio | 已注册全局点云（世界坐标系）|
| `/trajectory` | `nav_msgs/Path` | fast_lio | 运动轨迹历史路径 |
| `/slam/start` | `std_msgs/Bool` | lora_bridge | LoRa 遥控启动 SLAM |
| `/slam/stop` | `std_msgs/Bool` | lora_bridge | LoRa 遥控停止 SLAM |

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
