#!/bin/bash
# 宇树 L1 雷达 SLAM 一键启动脚本（FAST-LIO2 + Qt 全局地图）
set -e

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="/run/user/$(id -u)"
DRIVER_CTR="l1_driver"
# 用 Dockerfile 的 SHA256 前8位做镜像 tag，Dockerfile 变更时自动重建
DOCKER_HASH=$(sha256sum "$WORKSPACE/Dockerfile.lidar" | cut -c1-8)
IMAGE="lidar-ros:${DOCKER_HASH}"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

echo "================================================"
echo "   宇树 L1 LiDAR SLAM 启动脚本 (FAST-LIO2)"
echo "================================================"

# ── 检查串口 ─────────────────────────────────────────
if [ ! -e /dev/ttyUSB0 ]; then
    error "未检测到 /dev/ttyUSB0，请确认雷达已连接"
fi
info "雷达串口 /dev/ttyUSB0 已就绪"

# ── 构建镜像（Dockerfile 变更时自动重建）────────────
if ! docker image inspect $IMAGE &>/dev/null; then
    info "构建 Docker 镜像 ${IMAGE}（约5分钟）..."
    docker build -f "$WORKSPACE/Dockerfile.lidar" -t $IMAGE "$WORKSPACE" || error "镜像构建失败"
    # 清理旧版本镜像（保留当前）
    docker images lidar-ros --format "{{.Tag}}" | grep -v "${DOCKER_HASH}" | \
        xargs -I{} docker rmi "lidar-ros:{}" 2>/dev/null || true
    info "镜像构建完成"
fi

# ── 清理旧容器 ────────────────────────────────────────
info "清理旧容器..."
docker rm -f $DRIVER_CTR 2>/dev/null || true

# ── 启动容器 ──────────────────────────────────────────
info "启动容器..."
docker run -d --name $DRIVER_CTR \
    --network host \
    --device=/dev/ttyUSB0 \
    -e WAYLAND_DISPLAY=wayland-0 \
    -e XDG_RUNTIME_DIR=$RUNTIME_DIR \
    -e QT_QPA_PLATFORM=wayland \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v $RUNTIME_DIR/wayland-0:$RUNTIME_DIR/wayland-0 \
    -v "$WORKSPACE:/workspace" \
    $IMAGE \
    sleep infinity

# ── 编译所有包（colcon 增量编译，只重建有变化的）─────
info "编译工作空间..."
docker exec $DRIVER_CTR bash -c "
    source /opt/ros/jazzy/setup.bash &&
    cd /workspace &&
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
" || error "编译失败，请检查上方错误"
info "编译完成"

# ── 启动雷达驱动 ──────────────────────────────────────
info "启动 l1_driver 节点..."
docker exec -d $DRIVER_CTR bash -c "
    source /opt/ros/jazzy/setup.bash &&
    source /workspace/install/setup.bash &&
    ros2 launch l1_driver l1_driver.launch.py
"

sleep 2

# ── 启动 FAST-LIO2 SLAM ───────────────────────────────
info "启动 FAST-LIO2 SLAM 节点..."
docker exec -d $DRIVER_CTR bash -c "
    source /opt/ros/jazzy/setup.bash &&
    source /workspace/install/setup.bash &&
    ros2 launch fast_lio fast_lio.launch.py
"

sleep 1

# ── 启动 Qt 可视化（前台）────────────────────────────
info "启动 Qt 全局地图可视化窗口..."
docker exec $DRIVER_CTR bash -c "
    source /opt/ros/jazzy/setup.bash &&
    source /workspace/install/setup.bash &&
    /workspace/install/qt_visualizer/lib/qt_visualizer/qt_visualizer_node
" &
QT_PID=$!

# ── 完成 ─────────────────────────────────────────────
echo ""
echo "================================================"
info "全部服务已启动！"
echo "  数据流: l1_driver → FAST-LIO2 → Qt 全局地图"
echo "  Qt 窗口实时显示建图结果和运动轨迹"
echo "  按 Ctrl+C 停止所有服务"
echo "================================================"

cleanup() {
    echo ""
    info "正在停止所有服务..."
    kill $QT_PID 2>/dev/null || true
    docker rm -f $DRIVER_CTR 2>/dev/null || true
    info "已停止"
}
trap cleanup INT TERM

wait $QT_PID
