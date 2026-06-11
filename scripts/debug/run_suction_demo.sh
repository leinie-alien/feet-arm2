#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/zyy/task/arm/feet-arm2-main"
SERIAL_PORT="${1:-/dev/esp32_suction_c3}"
SERVICE_NAME="${2:-set_suction}"
SERVER_PID=""

cleanup() {
  if [[ -n "${SERVER_PID}" ]] && kill -0 "${SERVER_PID}" 2>/dev/null; then
    kill "${SERVER_PID}" 2>/dev/null || true
    wait "${SERVER_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

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

echo "Starting suction service node on ${SERIAL_PORT}..."
ros2 run suction_serial_bridge suction_service_node \
  --ros-args \
  -p serial_port:="${SERIAL_PORT}" \
  -p service_name:="${SERVICE_NAME}" \
  >/tmp/suction_service_node.log 2>&1 &
SERVER_PID=$!

sleep 1
if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
  echo "Failed to start suction service node." >&2
  echo "Log:" >&2
  cat /tmp/suction_service_node.log >&2 || true
  exit 1
fi

echo "Service node started. Log: /tmp/suction_service_node.log"
echo "Launching keyboard client. Press q to quit."

ros2 run suction_serial_bridge suction_keyboard_client \
  --ros-args \
  -p service_name:="${SERVICE_NAME}"
