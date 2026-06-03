#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/primarymage/WorkFile/esp_ws"
WS_DIR="${ROOT_DIR}/ros2_suction_ws"
SERIAL_PORT="${1:-/dev/esp32_suction_c3}"
SERVICE_NAME="${2:-set_suction}"
ROS_DOMAIN_ID_VALUE="${3:-0}"
RMW_IMPL="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Missing /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Missing ${WS_DIR}/install/setup.bash. Build the workspace first." >&2
  exit 1
fi

if [[ ! -e "${SERIAL_PORT}" ]]; then
  echo "Serial device not found: ${SERIAL_PORT}" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export RMW_IMPLEMENTATION="${RMW_IMPL}"

echo "Receiver host started."
echo "  serial_port=${SERIAL_PORT}"
echo "  service_name=${SERVICE_NAME}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

exec ros2 run suction_serial_bridge suction_service_node \
  --ros-args \
  -p serial_port:="${SERIAL_PORT}" \
  -p service_name:="${SERVICE_NAME}"
