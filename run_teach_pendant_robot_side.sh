#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$SCRIPT_DIR/install/setup.bash"

DEFAULT_ARM_PARAMS_FILE="$SCRIPT_DIR/src/arm2_task/config/params.yaml"
DEFAULT_DRIVER_PARAMS_FILE="$SCRIPT_DIR/src/dm_motor_sdk_ros/config/dm_motor_robot_driver.yaml"

ARM_PARAMS_FILE="$DEFAULT_ARM_PARAMS_FILE"
DRIVER_PARAMS_FILE="$DEFAULT_DRIVER_PARAMS_FILE"
READY_TIMEOUT=10

DRIVER_PID=""
INVERSE_DYNAMICS_PID=""
CLEANUP_RUNNING=0
ROS2_DAEMON_PATTERN='[r]os2cli\.daemon\.daemonize.*--name ros2-daemon'
declare -a ROS2_DAEMON_PIDS_BEFORE=()

usage() {
    cat <<USAGE
Usage: $(basename "$0") [options]

Robot-side startup for distributed teach pendant control:
1. Launch dm_motor_sdk_ros driver
2. Wait for /robot_driver/ready == true
3. Launch arm2_task inverse_dynamics_node

Run this script on the real robot controller device.

Options:
  --arm-params <file>     arm2_task parameter file
  --driver-params <file>  dm_motor_sdk_ros driver parameter file
  --ready-timeout <sec>   wait timeout for /robot_driver/ready (default: ${READY_TIMEOUT})
  -h, --help              show this help
USAGE
}

stop_process_group() {
    local pid="$1"
    local name="$2"

    [[ -n "${pid}" ]] || return 0

    echo "[INFO] Stopping ${name} (pgid=${pid})"
    kill -TERM -- "-${pid}" 2>/dev/null || true

    for _ in $(seq 1 20); do
        if ! kill -0 -- "-${pid}" 2>/dev/null; then
            wait "${pid}" 2>/dev/null || true
            return 0
        fi
        sleep 0.1
    done

    if kill -0 -- "-${pid}" 2>/dev/null; then
        echo "[WARN] ${name} did not stop after SIGTERM; sending SIGKILL to process group ${pid}"
        kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
}

launch_in_group() {
    local __resultvar="$1"
    shift

    setsid "$@" &
    local pid=$!
    printf -v "${__resultvar}" '%s' "${pid}"
}

record_existing_ros2_daemons() {
    mapfile -t ROS2_DAEMON_PIDS_BEFORE < <(pgrep -f "${ROS2_DAEMON_PATTERN}" || true)
}

stop_new_ros2_daemons() {
    local -a current_pids=()
    mapfile -t current_pids < <(pgrep -f "${ROS2_DAEMON_PATTERN}" || true)

    for pid in "${current_pids[@]}"; do
        local seen_before=0
        for old_pid in "${ROS2_DAEMON_PIDS_BEFORE[@]}"; do
            if [[ "${pid}" == "${old_pid}" ]]; then
                seen_before=1
                break
            fi
        done

        if (( ! seen_before )); then
            echo "[INFO] Stopping ros2-daemon (pid=${pid})"
            kill -TERM "${pid}" 2>/dev/null || true
            for _ in $(seq 1 20); do
                if ! kill -0 "${pid}" 2>/dev/null; then
                    break
                fi
                sleep 0.1
            done
            if kill -0 "${pid}" 2>/dev/null; then
                echo "[WARN] ros2-daemon did not stop after SIGTERM; sending SIGKILL to pid ${pid}"
                kill -KILL "${pid}" 2>/dev/null || true
            fi
        fi
    done
}

cleanup() {
    if (( CLEANUP_RUNNING )); then
        return
    fi
    CLEANUP_RUNNING=1

    local pids=(
        "${INVERSE_DYNAMICS_PID}"
        "${DRIVER_PID}"
    )
    local names=(
        "inverse_dynamics_node"
        "dm_motor_sdk_ros driver"
    )

    for i in "${!pids[@]}"; do
        stop_process_group "${pids[$i]}" "${names[$i]}"
    done

    stop_new_ros2_daemons
}

source_setup() {
    local setup_file="$1"
    set +u
    source "${setup_file}"
    set -u
}

assert_binary_runnable() {
    local binary="$1"
    local label="$2"
    local -a missing_libs=()

    if [[ ! -e "${binary}" ]]; then
        echo "[ERROR] ${label} executable not found: ${binary}" >&2
        return 1
    fi

    mapfile -t missing_libs < <(ldd "${binary}" 2>&1 | awk '/=> not found/ { print $1 }' || true)
    if (( ${#missing_libs[@]} > 0 )); then
        echo "[ERROR] ${label} has unresolved shared library dependencies:" >&2
        echo "        ${binary}" >&2
        printf '        %s\n' "${missing_libs[@]}" >&2
        echo "[ERROR] Rebuild the affected package(s) against the current environment before launching the real robot." >&2
        return 1
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
    echo "        Build the workspace first, e.g. colcon build --base-paths src --packages-select dm_motor_sdk_ros arm2_task" >&2
    exit 1
fi

if [[ ! -f "${ARM_PARAMS_FILE}" ]]; then
    echo "[ERROR] arm2_task params file not found: ${ARM_PARAMS_FILE}" >&2
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
record_existing_ros2_daemons

assert_binary_runnable "${SCRIPT_DIR}/install/dm_motor_sdk_ros/lib/dm_motor_sdk_ros/dm_motor_robot_driver_node" "dm_motor_robot_driver_node"
assert_binary_runnable "${SCRIPT_DIR}/install/arm2_task/lib/arm2_task/inverse_dynamics_node" "inverse_dynamics_node"

echo "[INFO] arm2_task params: ${ARM_PARAMS_FILE}"
echo "[INFO] driver params: ${DRIVER_PARAMS_FILE}"

echo "[INFO] Launching dm_motor_sdk_ros driver..."
launch_in_group DRIVER_PID ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py params_path:="${DRIVER_PARAMS_FILE}"

echo "[INFO] Waiting for /robot_driver/ready == true (timeout: ${READY_TIMEOUT}s)..."
if ! wait_for_ready_true "${READY_TIMEOUT}"; then
    echo "[ERROR] Timed out waiting for /robot_driver/ready == true" >&2
    exit 1
fi

echo "[INFO] Launching inverse_dynamics_node..."
launch_in_group INVERSE_DYNAMICS_PID ros2 launch arm2_task inverse_dynamics_launch.py params_path:="${ARM_PARAMS_FILE}"

echo "[INFO] Robot-side teach pendant stack is ready."
echo "[INFO] Start the pendant-side script on the teaching device once ROS 2 LAN discovery is configured."

wait -n "${DRIVER_PID}" "${INVERSE_DYNAMICS_PID}"
EXIT_CODE=$?
echo "[ERROR] One of the robot-side teach pendant processes exited unexpectedly" >&2
exit "${EXIT_CODE}"
