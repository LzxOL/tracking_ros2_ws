#!/usr/bin/env bash
# 启动完整的跟踪与机械臂控制系统
# 1. 启动视频流节点 (robot_video_client)
# 2. 检查视频流是否正常
# 3. 启动 Tracking 节点 (track_camera_front_min)
# 4. 启动 3D-TF 转换与机械臂控制节点 (points3d_tf_to_arm_base_node)

set -euo pipefail

# 获取脚本所在目录（工作空间根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ============================
# 自动 source 工作空间
# ============================
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "[INFO] 自动 source install/setup.bash"
    set +u
    source "${SCRIPT_DIR}/install/setup.bash"
    set -u
else
    echo "错误: 找不到 ${SCRIPT_DIR}/install/setup.bash"
    echo "请先编译: colcon build"
    exit 1
fi

# 添加组播路由，避免视频接收失败
echo "[INFO] 添加组播路由..."
sudo route add -net 224.5.6.7 netmask 255.255.255.255 dev eno1 || true

echo "=========================================="
echo "启动完整的跟踪与机械臂控制系统"
echo "=========================================="

# ============================
# 运行 run_node：统一启动全部流程
# ============================
echo "[1/1] 运行 run_node（会自动启动视频、Tracking、以及 points3d_tf_to_arm_base_node）..."
echo "=========================================="
echo "按 Ctrl+C 退出所有节点"
echo "=========================================="

ros2 run monte_controller_node run_node
