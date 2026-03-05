#!/bin/bash
# 在 ROS2 环境下启动 Web 上位机

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../tracking_with_cameara_ws" && pwd)"

# 允许本机完整 DDS 发现（避免仅 localhost 导致收不到图像）
: "${ROS_LOCALHOST_ONLY:=0}"
export ROS_LOCALHOST_ONLY

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "请先构建 tracking_with_cameara_ws："
  echo "  cd $WS_DIR && colcon build --packages-select track_on_ros2_srv track_on_ros2"
  exit 1
fi

source "$WS_DIR/install/setup.bash"
cd "$SCRIPT_DIR"
exec python app.py "$@"
