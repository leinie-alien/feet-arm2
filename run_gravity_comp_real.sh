#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$SCRIPT_DIR/install/setup.bash"
DEFAULT_PARAMS_FILE="$SCRIPT_DIR/src/arm2_task/config/params.yaml"
DEFAULT_DRIVER_PARAMS_FILE="$SCRIPT_DIR/src/dm_motor_sdk_ros/config/dm_motor_robot_driver.yaml"

PARAMS_FILE="$DEFAULT_PARAMS_FILE"
DRIVER_PARAMS_FILE="$DEFAULT_DRIVER_PARAMS_FILE"
READY_TIMEOUT=10
DRIVER_PID=""
APP_PID=""

usage() {
    cat <<USAGE
Usage: $(basename "$0") [options] [-- <extra ros args>]

One-click startup for real robot gravity compensation:
1. Launch dm_motor_sdk_ros driver
2. Wait for /robot_driver/ready
3. Run arm2_task gravity_comp_test_node

Options:
  --params <file>         arm2_task parameter file
  --driver-params <file>  dm_motor_sdk_ros driver parameter file
  --ready-timeout <sec>   wait timeout for /robot_driver/ready (default: ${READY_TIMEOUT})
  -h, --help              show this help

Anything after '--' is forwarded to gravity_comp_test_node.
USAGE
}

cleanup() {
    if [[ -n "${APP_PID}" ]] && kill -0 "${APP_PID}" 2>/dev/null; then
        echo "[INFO] Stopping gravity_comp_test_node (pid=${APP_PID})"
        kill "${APP_PID}" 2>/dev/null || true
        wait "${APP_PID}" 2>/dev/null || true
    fi

    if [[ -n "${DRIVER_PID}" ]] && kill -0 "${DRIVER_PID}" 2>/dev/null; then
        echo "[INFO] Stopping dm_motor_sdk_ros driver (pid=${DRIVER_PID})"
        kill "${DRIVER_PID}" 2>/dev/null || true
        wait "${DRIVER_PID}" 2>/dev/null || true
    fi

    if pgrep -f "/home/primarymage/WorkFile/arm/install/dm_motor_sdk_ros/lib/dm_motor_sdk_ros/dm_motor_robot_driver_node" >/dev/null 2>&1; then
        echo "[INFO] Cleaning up residual dm_motor_robot_driver_node processes"
        pkill -f "/home/primarymage/WorkFile/arm/install/dm_motor_sdk_ros/lib/dm_motor_sdk_ros/dm_motor_robot_driver_node" 2>/dev/null || true
    fi

    if pgrep -f "/home/primarymage/WorkFile/arm/install/arm2_task/lib/arm2_task/gravity_comp_test_node" >/dev/null 2>&1; then
        echo "[INFO] Cleaning up residual gravity_comp_test_node processes"
        pkill -f "/home/primarymage/WorkFile/arm/install/arm2_task/lib/arm2_task/gravity_comp_test_node" 2>/dev/null || true
    fi
}

wait_for_ready_true() {
    local timeout_sec="$1"
    local deadline=$((SECONDS + timeout_sec))

    while (( SECONDS < deadline )); do
        if [[ -n "${DRIVER_PID}" ]] && ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
            echo "[ERROR] dm_motor_sdk_ros driver exited before publishing /robot_driver/ready == true" >&2
            return 1
        fi

        if ros2 topic echo /robot_driver/ready std_msgs/msg/Bool \
            --once \
            --qos-reliability reliable \
            --qos-durability transient_local 2>/dev/null | grep -q "data: true"; then
            return 0
        fi

        sleep 0.2
    done

    return 1
}

source_setup() {
    local setup_file="$1"
    set +u
    source "${setup_file}"
    set -u
}

EXTRA_ROS_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --params)
            PARAMS_FILE="$2"
            shift 2
            ;;
        --driver-params)
            DRIVER_PARAMS_FILE="$2"
            shift 2
            ;;
        --ready-timeout)
            READY_TIMEOUT="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            EXTRA_ROS_ARGS=("$@")
            break
            ;;
        *)
            echo "[ERROR] Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "[ERROR] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi

if [[ ! -f "${WS_SETUP}" ]]; then
    echo "[ERROR] Workspace setup not found: ${WS_SETUP}" >&2
    echo "        Build the workspace first, e.g. colcon build --packages-select dm_motor_sdk_ros arm2_task" >&2
    exit 1
fi

if [[ ! -f "${PARAMS_FILE}" ]]; then
    echo "[ERROR] arm2_task params file not found: ${PARAMS_FILE}" >&2
    exit 1
fi

if [[ ! -f "${DRIVER_PARAMS_FILE}" ]]; then
    echo "[ERROR] dm_motor_sdk_ros driver params file not found: ${DRIVER_PARAMS_FILE}" >&2
    exit 1
fi

trap cleanup EXIT INT TERM

cd "${SCRIPT_DIR}"

source_setup "${ROS_SETUP}"
source_setup "${WS_SETUP}"

echo "[INFO] arm2_task params: ${PARAMS_FILE}"
echo "[INFO] driver params: ${DRIVER_PARAMS_FILE}"
echo "[INFO] Launching dm_motor_sdk_ros driver..."
ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py params_path:="${DRIVER_PARAMS_FILE}" &
DRIVER_PID=$!

echo "[INFO] Waiting for /robot_driver/ready == true (timeout: ${READY_TIMEOUT}s)..."
if ! wait_for_ready_true "${READY_TIMEOUT}"; then
    echo "[ERROR] Timed out waiting for /robot_driver/ready == true" >&2
    exit 1
fi

echo "[INFO] Starting gravity_comp_test_node..."
ros2 run arm2_task gravity_comp_test_node --ros-args --params-file "${PARAMS_FILE}" "${EXTRA_ROS_ARGS[@]}" &
APP_PID=$!
wait "${APP_PID}"
APP_RC=$?
APP_PID=""
exit "${APP_RC}"
