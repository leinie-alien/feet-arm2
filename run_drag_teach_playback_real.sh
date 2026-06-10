#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$SCRIPT_DIR/install/setup.bash"
DEFAULT_ARM_PARAMS_FILE="$SCRIPT_DIR/src/arm2_task/config/params.yaml"
DEFAULT_DRIVER_PARAMS_FILE="$SCRIPT_DIR/src/dm_motor_sdk_ros/config/dm_motor_robot_driver.yaml"

ARM_PARAMS_FILE="$DEFAULT_ARM_PARAMS_FILE"
DRIVER_PARAMS_FILE="$DEFAULT_DRIVER_PARAMS_FILE"
INPUT_FILE=""
READY_TIMEOUT=10

DRIVER_PID=""
INVERSE_DYNAMICS_PID=""
PLAYBACK_PID=""

usage() {
    cat <<USAGE
Usage: $(basename "$0") [options]

Drag-teach playback startup:
1. Launch dm_motor_sdk_ros driver
2. Wait for /robot_driver/ready == true
3. Launch inverse_dynamics_node
4. Launch teach_drag_playback_node to publish recorded RobotState targets

Options:
  --arm-params <file>     arm2_task parameter file
  --driver-params <file>  dm_motor_sdk_ros driver parameter file
  --input-file <file>     trajectory CSV input path
  --ready-timeout <sec>   wait timeout for /robot_driver/ready (default: ${READY_TIMEOUT})
  -h, --help              show this help
USAGE
}

stop_process_group() {
    local pid="$1"
    local name="$2"
    [[ -n "${pid}" ]] || return 0
    kill -TERM -- "-${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
        if ! kill -0 -- "-${pid}" 2>/dev/null; then
            wait "${pid}" 2>/dev/null || true
            return 0
        fi
        sleep 0.1
    done
    kill -KILL -- "-${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
}

cleanup() {
    stop_process_group "${PLAYBACK_PID}" "teach_drag_playback_node"
    stop_process_group "${INVERSE_DYNAMICS_PID}" "inverse_dynamics_node"
    stop_process_group "${DRIVER_PID}" "dm_motor_sdk_ros driver"
}

launch_in_group() {
    local __resultvar="$1"
    shift
    setsid "$@" &
    local pid=$!
    printf -v "${__resultvar}" '%s' "${pid}"
}

source_setup() {
    local setup_file="$1"
    set +u
    source "${setup_file}"
    set -u
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

while [[ $# -gt 0 ]]; do
    case "$1" in
        --arm-params)
            ARM_PARAMS_FILE="$2"
            shift 2
            ;;
        --driver-params)
            DRIVER_PARAMS_FILE="$2"
            shift 2
            ;;
        --input-file)
            INPUT_FILE="$2"
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
    exit 1
fi

if [[ ! -f "${ARM_PARAMS_FILE}" ]]; then
    echo "[ERROR] arm2_task params file not found: ${ARM_PARAMS_FILE}" >&2
    exit 1
fi

if [[ ! -f "${DRIVER_PARAMS_FILE}" ]]; then
    echo "[ERROR] driver params file not found: ${DRIVER_PARAMS_FILE}" >&2
    exit 1
fi

trap cleanup EXIT INT TERM

cd "${SCRIPT_DIR}"
source_setup "${ROS_SETUP}"
source_setup "${WS_SETUP}"
export ARM_WORKSPACE_DIR="${SCRIPT_DIR}"

echo "[INFO] Launching dm_motor_sdk_ros driver..."
launch_in_group DRIVER_PID ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py params_path:="${DRIVER_PARAMS_FILE}"

echo "[INFO] Waiting for /robot_driver/ready ..."
if ! wait_for_ready_true "${READY_TIMEOUT}"; then
    echo "[ERROR] Timed out waiting for /robot_driver/ready == true" >&2
    exit 1
fi

echo "[INFO] Launching inverse_dynamics_node..."
launch_in_group INVERSE_DYNAMICS_PID ros2 launch arm2_task inverse_dynamics_launch.py params_path:="${ARM_PARAMS_FILE}"

PLAYBACK_CMD=(ros2 run arm2_task teach_drag_playback_node --ros-args --params-file "${ARM_PARAMS_FILE}")
if [[ -n "${INPUT_FILE}" ]]; then
    PLAYBACK_CMD+=(-p "teach_drag_playback.input_file:=${INPUT_FILE}")
fi

echo "[INFO] Launching teach_drag_playback_node..."
launch_in_group PLAYBACK_PID "${PLAYBACK_CMD[@]}"

wait "${PLAYBACK_PID}"
