#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="$(command -v python3)"
TIMEOUT_BIN="$(command -v timeout || true)"
TF_SCRIPT="${WS_DIR}/src/monte_controller_node/api_example/test_get_robot_tf.py"
CONFIG_DIR="${WS_DIR}/config"
ROBOT_CONFIG_YAML="${WS_DIR}/src/monte_controller_node/config/config.yaml"

usage() {
  echo "用法: $0 --robot-name <名称> [--robot-ip <ip:端口>]"
  echo "  示例: $0 -r mt0212"
  echo "  示例: $0 -r mt0212 --robot-ip 192.168.11.3:50051"
}

read_robot_ip_from_config() {
  local ip=""
  if [[ -f "${ROBOT_CONFIG_YAML}" ]]; then
    ip="$(
      awk '
        BEGIN {in_robot=0}
        /^[[:space:]]*robot:[[:space:]]*$/ {in_robot=1; next}
        in_robot && /^[^[:space:]]/ {in_robot=0}
        in_robot && /^[[:space:]]*ip:[[:space:]]*/ {
          line=$0
          sub(/^[[:space:]]*ip:[[:space:]]*/, "", line)
          gsub(/"/, "", line)
          gsub(/'\''/, "", line)
          gsub(/[[:space:]]+$/, "", line)
          print line
          exit
        }
      ' "${ROBOT_CONFIG_YAML}" || true
    )"
  fi
  echo "${ip}"
}

extract_suffix_from_robot_name() {
  local name="$1"
  # Naming rule:
  #   mt029  -> 9
  #   mt0212 -> 12
  # i.e. use the numeric part after "mt02".
  if [[ "${name}" =~ ^mt02([0-9]+)$ ]]; then
    local raw="${BASH_REMATCH[1]}"
    local normalized=$((10#${raw}))
    echo "${normalized}"
    return 0
  fi
  return 1
}

normalize_robot_ip() {
  local ip="$1"
  if [[ -z "${ip}" ]]; then
    echo ""
    return 0
  fi
  # If only host is provided, default to RobotLib gRPC port 50051.
  if [[ "${ip}" != *:* ]]; then
    echo "${ip}:50051"
  else
    echo "${ip}"
  fi
}

split_host_port() {
  local ip_port="$1"
  local host="${ip_port%:*}"
  local port="${ip_port##*:}"
  echo "${host}" "${port}"
}

tcp_connect_ok() {
  local host="$1"
  local port="$2"
  local nc_bin
  nc_bin="$(command -v nc || true)"

  if [[ -n "${nc_bin}" ]]; then
    "${nc_bin}" -z -w 2 "${host}" "${port}" >/dev/null 2>&1
    return $?
  fi

  # Fallback without nc.
  timeout 2 bash -lc "exec 3<>/dev/tcp/${host}/${port}" >/dev/null 2>&1
}

run_one_transform() {
  local parent_frame="$1"
  local child_frame="$2"
  local yaml_key="$3"
  local output_file="$4"
  local attempts=3
  local i=1

  echo "正在生成 ${output_file} (${parent_frame} -> ${child_frame})"
  while [[ ${i} -le ${attempts} ]]; do
    if "${TIMEOUT_BIN}" --signal=INT --kill-after=1s 12s \
      "${PYTHON_BIN}" "${TF_SCRIPT}" \
        --robot-ip "${ROBOT_IP}" \
        --parent-id "${parent_frame}" \
        --child-id "${child_frame}" \
        --yaml-key "${yaml_key}" \
        --output-file "${CONFIG_DIR}/${output_file}" \
        --no-prompt; then
      return 0
    fi

    ec=$?
    if [[ ${i} -lt ${attempts} ]]; then
      echo "警告: 第 ${i}/${attempts} 次尝试获取 TF ${parent_frame} -> ${child_frame} 失败，重试中..."
      sleep 1
    else
      echo "错误: 获取 TF ${parent_frame} -> ${child_frame} 失败，退出码=${ec}"
      exit "${ec}"
    fi
    i=$((i + 1))
  done
}

update_config_yaml() {
  local suffix="$1"
  local config_file="${ROBOT_CONFIG_YAML}"
  local track_config_file="${WS_DIR}/src/track_on_ros2/config/config.yaml"

  # Update monte_controller_node/config/config.yaml
  if [[ -f "${config_file}" ]]; then
    echo "=== 正在更新 ${config_file} 中的后缀为 ${suffix} ==="
    sed -i "s/joint_lt_sensor_rgbd_[0-9]\+\.txt/joint_lt_sensor_rgbd_${suffix}.txt/g" "${config_file}"
    sed -i "s/joint_rt_sensor_rgbd_[0-9]\+\.txt/joint_rt_sensor_rgbd_${suffix}.txt/g" "${config_file}"
    echo "配置文件更新成功"
  else
    echo "警告: 配置文件未找到: ${config_file}，跳过配置更新"
  fi

  # Update track_on_ros2/config/config.yaml (intrinsics files)
  # Note: left and right cameras may have different suffixes
  if [[ -f "${track_config_file}" ]]; then
    echo "=== Updating ${track_config_file} with suffix ${suffix} ==="
    # Update left camera intrinsics (lhand_front)
    sed -i "s/camera_lhand_front_intrinsics_02_[0-9]\{1,\}\.txt/camera_lhand_front_intrinsics_02_${suffix}.txt/g" "${track_config_file}"
    # Update right camera intrinsics (rhand_front)
    sed -i "s/camera_rhand_front_intrinsics_02_[0-9]\{1,\}\.txt/camera_rhand_front_intrinsics_02_${suffix}.txt/g" "${track_config_file}"
    echo "Track 配置文件更新成功"
  else
    echo "警告: Track 配置文件未找到: ${track_config_file}，跳过配置更新"
  fi
}

get_current_domain_id() {
  # Try to get current ROS_DOMAIN_ID from environment
  echo "${ROS_DOMAIN_ID:-0}"
}

# Function to fetch camera intrinsics from a specific domain
fetch_camera_intrinsics() {
  local domain_id="$1"
  local camera_topic="$2"
  local output_file="$3"
  local camera_name="$4"
  
  local temp_domain_id
  temp_domain_id=$(get_current_domain_id)
  
  echo "正在获取 ${camera_name} 内参 (domain ${domain_id})..."
  
  # Save original domain and set new one
  export ROS_DOMAIN_ID="${domain_id}"
  
  # Wait longer for ROS to discover topics (at least 3 seconds)
  echo "  等待 ROS 发现话题 (domain ${domain_id})..."
  sleep 3
  
  # First check if topic exists
  local topic_list
  topic_list=$(ros2 topic list 2>/dev/null || echo "")
  if ! echo "${topic_list}" | grep -q "${camera_topic}"; then
    echo "  -> 警告: 话题 ${camera_topic} 在 domain ${domain_id} 中未找到"
    echo "  -> 可用的 camera_info 话题:"
    echo "${topic_list}" | grep -E "camera_info" | head -10
    # Restore original domain_id
    export ROS_DOMAIN_ID="${temp_domain_id}"
    return 1
  fi
  
  # Try to get camera_info, timeout after 15 seconds, get only one message
  local camera_info
  if camera_info=$(timeout 15 ros2 topic echo "${camera_topic}" --once 2>&1); then
    # Parse the intrinsics using Python YAML parser (more robust)
    local parsed_info
    parsed_info=$(echo "${camera_info}" | python3 -c "
import sys
import yaml
try:
    content = sys.stdin.read().split('---')[0]
    data = yaml.safe_load(content)
    k = data.get('k', [])
    d = data.get('d', [])
    print(f'fx={k[0] if len(k) > 0 else \"\"}')
    print(f'cx={k[2] if len(k) > 2 else \"\"}')
    print(f'fy={k[4] if len(k) > 4 else \"\"}')
    print(f'cy={k[5] if len(k) > 5 else \"\"}')
    print(f'width={data.get(\"width\", \"\")}')
    print(f'height={data.get(\"height\", \"\")}')
    print(f'distortion_model={data.get(\"distortion_model\", \"\")}')
    print(f'd0={d[0] if len(d) > 0 else \"\"}')
    print(f'd1={d[1] if len(d) > 1 else \"\"}')
    print(f'd2={d[2] if len(d) > 2 else \"\"}')
    print(f'd3={d[3] if len(d) > 3 else \"\"}')
    print(f'd4={d[4] if len(d) > 4 else \"\"}')
except Exception as e:
    print(f'error={e}', file=sys.stderr)
" 2>&1)
    
    if echo "${parsed_info}" | grep -q "error="; then
      echo "  -> 警告: YAML 解析失败: ${parsed_info}"
      export ROS_DOMAIN_ID="${temp_domain_id}"
      return 1
    fi
    
    # Extract values from parsed output
    local fx cx fy cy width height distortion_model d0 d1 d2 d3 d4
    fx=$(echo "${parsed_info}" | grep -oP '^fx=\K[0-9.]+' | head -1)
    cx=$(echo "${parsed_info}" | grep -oP '^cx=\K[0-9.]+' | head -1)
    fy=$(echo "${parsed_info}" | grep -oP '^fy=\K[0-9.]+' | head -1)
    cy=$(echo "${parsed_info}" | grep -oP '^cy=\K[0-9.]+' | head -1)
    width=$(echo "${parsed_info}" | grep -oP '^width=\K[0-9]+' | head -1)
    height=$(echo "${parsed_info}" | grep -oP '^height=\K[0-9]+' | head -1)
    distortion_model=$(echo "${parsed_info}" | grep -oP '^distortion_model=\K\w+' | head -1)
    d0=$(echo "${parsed_info}" | grep -oP '^d0=\K[0-9.e-]+' | head -1)
    d1=$(echo "${parsed_info}" | grep -oP '^d1=\K[0-9.e-]+' | head -1)
    d2=$(echo "${parsed_info}" | grep -oP '^d2=\K[0-9.e-]+' | head -1)
    d3=$(echo "${parsed_info}" | grep -oP '^d3=\K[0-9.e-]+' | head -1)
    d4=$(echo "${parsed_info}" | grep -oP '^d4=\K[0-9.e-]+' | head -1)
    
    # Debug output
    echo "  -> 已解析: fx=${fx}, cx=${cx}, fy=${fy}, cy=${cy}, width=${width}, height=${height}"
    
    # Validate we got valid numbers
    if [[ -z "${fx}" || -z "${cx}" || -z "${fy}" ]]; then
      echo "  -> 警告: 无法从 ${camera_topic} 解析内参"
      echo "  -> 原始输出: ${camera_info}"
      # Restore original domain_id
      export ROS_DOMAIN_ID="${temp_domain_id}"
      return 1
    fi
    
    # Create the intrinsics file
    cat > "${CONFIG_DIR}/${output_file}" << EOF
# Camera intrinsics for ${camera_name}
# Format: K matrix (3x3)

K:
${fx}      0.0                     ${cx}
0.0                    ${fy}       ${cy}
0.0                    0.0         1.0

# Camera parameters:
# fx = ${fx}
# fy = ${fy}
# cx = ${cx}
# cy = ${cy}
# width  = ${width}
# height = ${height}
# distortion_model = ${distortion_model}
# distortion_coeffs = [
#     ${d0},
#     ${d1},
#     ${d2},
#     ${d3},
#     ${d4}
# ]
EOF
    echo "  -> 已保存到 ${CONFIG_DIR}/${output_file}"
  else
    echo "  -> 警告: 无法从话题 ${camera_topic} 获取 ${camera_name} 的 camera_info"
  fi
  
  # Restore original domain_id
  export ROS_DOMAIN_ID="${temp_domain_id}"
  sleep 1
}

# Ask user for target domain_id and fetch camera intrinsics
fetch_all_camera_intrinsics() {
  local current_domain
  current_domain=$(get_current_domain_id)
  
  echo ""
  echo "=== 获取相机内参 ==="
  echo "当前 ROS_DOMAIN_ID: ${current_domain}"
  echo ""
  read -p "请输入目标机器人的 domain_id 以获取相机内参（直接回车跳过）: " target_domain
  
  if [[ -z "${target_domain}" ]]; then
    echo "已跳过获取相机内参。"
    return 0
  fi
  
  # Validate domain_id is a number
  if ! [[ "${target_domain}" =~ ^[0-9]+$ ]]; then
    echo "错误: 无效的 domain_id '${target_domain}'，必须是数字"
    return 1
  fi
  
  # Fetch intrinsics for each camera
  # Left hand camera
  fetch_camera_intrinsics "${target_domain}" "/left/color/camera_info" "camera_lhand_front_intrinsics_02_${ROBOT_SUFFIX}.txt" "左手相机"
  
  # Right hand camera
  fetch_camera_intrinsics "${target_domain}" "/right/color/camera_info" "camera_rhand_front_intrinsics_02_${ROBOT_SUFFIX}.txt" "右手相机"
  
  echo ""
  echo "相机内参获取完成。"
}

ROBOT_NAME=""
ROBOT_IP=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -r|--robot-name)
      ROBOT_NAME="${2:-}"
      shift 2
      ;;
    --robot-ip)
      ROBOT_IP="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误: 未知参数: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ -z "${ROBOT_NAME}" ]]; then
  echo "错误: 必须指定 --robot-name 参数"
  usage
  exit 2
fi

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误: 未找到 python3"
  exit 2
fi

if [[ -z "${TIMEOUT_BIN}" ]]; then
  echo "错误: 未找到 'timeout' 命令 (coreutils)。请先安装。"
  exit 2
fi

if [[ ! -f "${TF_SCRIPT}" ]]; then
  echo "错误: 未找到 TF 脚本: ${TF_SCRIPT}"
  exit 2
fi

if ! ROBOT_SUFFIX="$(extract_suffix_from_robot_name "${ROBOT_NAME}")"; then
  echo "错误: 无法从机器人名称 '${ROBOT_NAME}' 提取后缀"
  echo "       期望格式: mt02<编号>, 例如: mt029 -> 9, mt0212 -> 12"
  exit 2
fi

if [[ -z "${ROBOT_IP}" ]]; then
  ROBOT_IP="$(read_robot_ip_from_config)"
fi
ROBOT_IP="$(normalize_robot_ip "${ROBOT_IP}")"
if [[ -z "${ROBOT_IP}" ]]; then
  echo "错误: 机器人 IP 为空。请通过 --robot-ip <ip:端口> 参数指定，或在 ${ROBOT_CONFIG_YAML} 中设置 robot.ip"
  exit 2
fi

read -r ROBOT_HOST ROBOT_PORT <<<"$(split_host_port "${ROBOT_IP}")"
if [[ -z "${ROBOT_HOST}" || -z "${ROBOT_PORT}" ]]; then
  echo "错误: 无效的机器人 IP: ${ROBOT_IP}"
  exit 2
fi

if ! tcp_connect_ok "${ROBOT_HOST}" "${ROBOT_PORT}"; then
  echo "错误: 无法连接到机器人 TCP 端点 ${ROBOT_HOST}:${ROBOT_PORT}"
  echo "       请检查机器人电源/网络，以及 gRPC 服务是否在端口 ${ROBOT_PORT} 上监听"
  exit 3
fi

mkdir -p "${CONFIG_DIR}"

echo "机器人名称: ${ROBOT_NAME}"
echo "机器人后缀: ${ROBOT_SUFFIX}"
echo "机器人IP: ${ROBOT_IP}"
echo "输出目录: ${CONFIG_DIR}"

echo "=== 正在生成手腕到相机的外参文件 ==="
run_one_transform \
  "link_l7_wrist_roll" \
  "link_lt_sensor_rgbd" \
  "joint_lt_sensor_rgbd" \
  "joint_lt_sensor_rgbd_${ROBOT_SUFFIX}.txt"

run_one_transform \
  "link_r7_wrist_roll" \
  "link_rt_sensor_rgbd" \
  "joint_rt_sensor_rgbd" \
  "joint_rt_sensor_rgbd_${ROBOT_SUFFIX}.txt"

update_config_yaml "${ROBOT_SUFFIX}"

# Fetch camera intrinsics (interactive)
fetch_all_camera_intrinsics

echo "完成。生成的文件:"
echo "  - ${CONFIG_DIR}/joint_lt_sensor_rgbd_${ROBOT_SUFFIX}.txt"
echo "  - ${CONFIG_DIR}/joint_rt_sensor_rgbd_${ROBOT_SUFFIX}.txt"
if [[ -f "${CONFIG_DIR}/camera_lhand_front_intrinsics_02_${ROBOT_SUFFIX}.txt" ]]; then
  echo "  - ${CONFIG_DIR}/camera_lhand_front_intrinsics_02_${ROBOT_SUFFIX}.txt"
fi
if [[ -f "${CONFIG_DIR}/camera_rhand_front_intrinsics_02_${ROBOT_SUFFIX}.txt" ]]; then
  echo "  - ${CONFIG_DIR}/camera_rhand_front_intrinsics_02_${ROBOT_SUFFIX}.txt"
fi
if [[ -f "${CONFIG_DIR}/camera_head_front_intrinsics_02_${ROBOT_SUFFIX}.txt" ]]; then
  echo "  - ${CONFIG_DIR}/camera_head_front_intrinsics_02_${ROBOT_SUFFIX}.txt"
fi
