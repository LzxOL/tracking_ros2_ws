#!/usr/bin/env bash
# 启动完整的跟踪与机械臂控制系统
# 1. 启动视频流节点 (robot_video_client)
# 2. 检查视频流是否正常
# 3. 启动 Tracking 节点 (track_camera_front_min)
# 4. 启动 3D-TF 转换与机械臂控制节点 (points3d_tf_to_arm_base_node)
#
# 参数：
#   --web-ui, -w    同时启动 Web UI（可选）
#
# 用法：
#   ./tracking_with_arm_control.sh          # 仅启动跟踪系统
#   ./tracking_with_arm_control.sh --web-ui  # 启动跟踪系统 + Web UI

set -euo pipefail

# ============================
# 解析参数
# ============================
START_WEB_UI=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --web-ui|-w)
            START_WEB_UI=true
            shift
            ;;
        -h|--help)
            echo "用法: $0 [--web-ui|-w]"
            echo ""
            echo "参数:"
            echo "  --web-ui 同时启动 Web UI, -w   "
            exit ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--web-ui|-w]"
            exit 1
            ;;
    esac
done

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
# 可选：启动 Web UI
# ============================
if [ "$START_WEB_UI" = true ]; then
    echo "[INFO] 正在启动 Web UI..."
    WEB_UI_DIR="${SCRIPT_DIR}/src/tracking_web_ui"
    if [ -f "${WEB_UI_DIR}/install/setup.bash" ]; then
        (
            set +u
            source "${WEB_UI_DIR}/install/setup.bash"
            set -u
            cd "$WEB_UI_DIR"
            python app.py
        ) &
        WEB_UI_PID=$!
        echo "[INFO] Web UI 已启动 (PID: $WEB_UI_PID)，访问 http://localhost:8080"
        echo "[INFO] 注意：关闭主节点后 Web UI 会自动退出"
    else
        echo "[WARNING] 找不到 Web UI 安装目录: ${WEB_UI_DIR}/install/setup.bash"
        echo "[WARNING] 跳过启动 Web UI，请先编译: cd ${SCRIPT_DIR} && colcon build --packages-select track_on_ros2_srv track_on_ros2"
    fi
fi

# ============================
# 运行 run_node：统一启动全部流程
# ============================
echo "[1/1] 运行 run_node（会自动启动视频、Tracking、以及 points3d_tf_to_arm_base_node）..."
echo "=========================================="
echo "按 Ctrl+C 退出所有节点"
echo "=========================================="

ros2 run monte_controller_node run_node
