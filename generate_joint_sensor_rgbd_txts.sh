#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="$(command -v python3)"
TIMEOUT_BIN="$(command -v timeout || true)"
TF_SCRIPT="${WS_DIR}/src/monte_controller_node/api_example/test_get_robot_tf.py"
CONFIG_DIR="${WS_DIR}/config"
ROBOT_CONFIG_YAML="${WS_DIR}/src/monte_controller_node/config/config.yaml"

usage() {
  echo "Usage: $0 --robot-name <name> [--robot-ip <ip:port>]"
  echo "  Example: $0 -r mt0212"
  echo "  Example: $0 -r mt0212 --robot-ip 192.168.11.3:50051"
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

  echo "Generating ${output_file} (${parent_frame} -> ${child_frame})"
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
      echo "WARN: attempt ${i}/${attempts} failed for TF ${parent_frame} -> ${child_frame}, retrying..."
      sleep 1
    else
      echo "ERROR: failed to fetch TF ${parent_frame} -> ${child_frame}, exit=${ec}"
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
    echo "=== Updating ${config_file} with suffix ${suffix} ==="
    sed -i "s/joint_lt_sensor_rgbd_[0-9]\+\.txt/joint_lt_sensor_rgbd_${suffix}.txt/g" "${config_file}"
    sed -i "s/joint_rt_sensor_rgbd_[0-9]\+\.txt/joint_rt_sensor_rgbd_${suffix}.txt/g" "${config_file}"
    echo "Config updated successfully"
  else
    echo "WARN: config file not found: ${config_file}, skipping config update"
  fi

  # Update track_on_ros2/config/config.yaml (intrinsics files)
  # Note: left and right cameras may have different suffixes
  if [[ -f "${track_config_file}" ]]; then
    echo "=== Updating ${track_config_file} with suffix ${suffix} ==="
    # Update left camera intrinsics (lhand_front)
    sed -i "s/camera_lhand_front_intrinsics_02_[0-9]\{1,\}\.txt/camera_lhand_front_intrinsics_02_${suffix}.txt/g" "${track_config_file}"
    # Update right camera intrinsics (rhand_front)
    sed -i "s/camera_rhand_front_intrinsics_02_[0-9]\{1,\}\.txt/camera_rhand_front_intrinsics_02_${suffix}.txt/g" "${track_config_file}"
    echo "Track config updated successfully"
  else
    echo "WARN: track config file not found: ${track_config_file}, skipping config update"
  fi
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
      echo "ERROR: Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ -z "${ROBOT_NAME}" ]]; then
  echo "ERROR: --robot-name is required"
  usage
  exit 2
fi

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "ERROR: python3 not found"
  exit 2
fi

if [[ -z "${TIMEOUT_BIN}" ]]; then
  echo "ERROR: 'timeout' command not found (coreutils). Please install it first."
  exit 2
fi

if [[ ! -f "${TF_SCRIPT}" ]]; then
  echo "ERROR: TF script not found: ${TF_SCRIPT}"
  exit 2
fi

if ! ROBOT_SUFFIX="$(extract_suffix_from_robot_name "${ROBOT_NAME}")"; then
  echo "ERROR: cannot derive suffix from robot name '${ROBOT_NAME}'"
  echo "       expected format: mt02<index>, e.g. mt029 -> 9, mt0212 -> 12"
  exit 2
fi

if [[ -z "${ROBOT_IP}" ]]; then
  ROBOT_IP="$(read_robot_ip_from_config)"
fi
ROBOT_IP="$(normalize_robot_ip "${ROBOT_IP}")"
if [[ -z "${ROBOT_IP}" ]]; then
  echo "ERROR: robot ip is empty. Pass --robot-ip <ip:port> or set robot.ip in ${ROBOT_CONFIG_YAML}"
  exit 2
fi

read -r ROBOT_HOST ROBOT_PORT <<<"$(split_host_port "${ROBOT_IP}")"
if [[ -z "${ROBOT_HOST}" || -z "${ROBOT_PORT}" ]]; then
  echo "ERROR: invalid robot ip: ${ROBOT_IP}"
  exit 2
fi

if ! tcp_connect_ok "${ROBOT_HOST}" "${ROBOT_PORT}"; then
  echo "ERROR: cannot connect to robot tcp endpoint ${ROBOT_HOST}:${ROBOT_PORT}"
  echo "       please check robot power/network and whether gRPC service is listening on port ${ROBOT_PORT}"
  exit 3
fi

mkdir -p "${CONFIG_DIR}"

echo "robot-name: ${ROBOT_NAME}"
echo "robot-suffix: ${ROBOT_SUFFIX}"
echo "robot-ip: ${ROBOT_IP}"
echo "output-dir: ${CONFIG_DIR}"

echo "=== Generating wrist-to-camera extrinsics txt ==="
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

echo "Done. Generated files:"
echo "  - ${CONFIG_DIR}/joint_lt_sensor_rgbd_${ROBOT_SUFFIX}.txt"
echo "  - ${CONFIG_DIR}/joint_rt_sensor_rgbd_${ROBOT_SUFFIX}.txt"
