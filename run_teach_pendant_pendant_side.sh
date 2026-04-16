#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$SCRIPT_DIR/install/setup.bash"

DEFAULT_ARM_PARAMS_FILE="$SCRIPT_DIR/src/arm2_task/config/params.yaml"
DEFAULT_TEACH_CONFIG_FILE="$SCRIPT_DIR/src/ftservo_hls3625_teach/config/servo_bus.yaml"

ARM_PARAMS_FILE="$DEFAULT_ARM_PARAMS_FILE"
TEACH_CONFIG_FILE="$DEFAULT_TEACH_CONFIG_FILE"
READY_TIMEOUT=10

TEACH_STATE_PUBLISHER_PID=""
TEACH_BRIDGE_PID=""
TEMP_TEACH_CONFIG_FILE=""
CLEANUP_RUNNING=0
ROS2_DAEMON_PATTERN='[r]os2cli\.daemon\.daemonize.*--name ros2-daemon'
declare -a ROS2_DAEMON_PIDS_BEFORE=()

usage() {
    cat <<USAGE
Usage: $(basename "$0") [options]

Pendant-side startup for distributed teach pendant control:
1. Launch ftservo_hls3625_teach bus_state_publisher
2. Wait for local /joint_states
3. Wait for remote /robot_driver/ready == true
4. Launch arm2_task teach_pendant_follow_node

Run this script on the teaching device.

Options:
  --arm-params <file>     arm2_task parameter file
  --teach-config <file>   ftservo_hls3625_teach config YAML
  --ready-timeout <sec>   wait timeout for /joint_states and /robot_driver/ready (default: ${READY_TIMEOUT})
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
        "${TEACH_BRIDGE_PID}"
        "${TEACH_STATE_PUBLISHER_PID}"
    )
    local names=(
        "teach_pendant_follow_node"
        "ftservo_hls3625_teach bus_state_publisher"
    )

    for i in "${!pids[@]}"; do
        stop_process_group "${pids[$i]}" "${names[$i]}"
    done

    stop_new_ros2_daemons

    if [[ -n "${TEMP_TEACH_CONFIG_FILE}" && -f "${TEMP_TEACH_CONFIG_FILE}" ]]; then
        rm -f "${TEMP_TEACH_CONFIG_FILE}"
    fi
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
        echo "[ERROR] Rebuild the affected package(s) against the current environment before launching the teach pendant stack." >&2
        return 1
    fi
}

extract_teach_port() {
    local config_file="$1"
    sed -n "s/^[[:space:]]*port:[[:space:]]*['\"]\\{0,1\\}\\([^'\"]*\\)['\"]\\{0,1\\}[[:space:]]*$/\\1/p" "${config_file}" | head -n 1
}

find_serial_candidates() {
    local candidate
    local patterns=(
        "/dev/serial/by-id/*"
        "/dev/ttyUSB*"
        "/dev/ttyACM*"
    )

    for pattern in "${patterns[@]}"; do
        for candidate in ${pattern}; do
            [[ -e "${candidate}" ]] || continue
            printf '%s\n' "${candidate}"
        done
    done
}

prepare_teach_config() {
    local configured_port
    configured_port="$(extract_teach_port "${TEACH_CONFIG_FILE}")"

    if [[ -z "${configured_port}" ]]; then
        echo "[WARN] Could not find a 'port' entry in ${TEACH_CONFIG_FILE}; using file as-is."
        return 0
    fi

    if [[ -e "${configured_port}" ]]; then
        echo "[INFO] Using teach serial port from config: ${configured_port}"
        return 0
    fi

    mapfile -t serial_candidates < <(find_serial_candidates)

    if (( ${#serial_candidates[@]} == 1 )); then
        local detected_port="${serial_candidates[0]}"
        TEMP_TEACH_CONFIG_FILE="$(mktemp /tmp/teach_servo_bus.XXXXXX.yaml)"
        sed "s|^\([[:space:]]*port:[[:space:]]*['\"]\\{0,1\\}\)[^'\"]*\(['\"]\\{0,1\\}[[:space:]]*$\)|\1${detected_port}\2|" \
            "${TEACH_CONFIG_FILE}" > "${TEMP_TEACH_CONFIG_FILE}"
        TEACH_CONFIG_FILE="${TEMP_TEACH_CONFIG_FILE}"
        echo "[WARN] Configured teach port ${configured_port} is missing."
        echo "[WARN] Falling back to detected serial device: ${detected_port}"
        echo "[WARN] To restore the stable path, install src/ftservo_hls3625_teach/udev/99-ftservo-hls3625.rules and replug the adapter."
        return 0
    fi

    echo "[ERROR] Configured teach port does not exist: ${configured_port}" >&2
    if (( ${#serial_candidates[@]} == 0 )); then
        echo "[ERROR] No serial candidates were detected under /dev/serial/by-id, /dev/ttyUSB*, or /dev/ttyACM*." >&2
        echo "[ERROR] Check that the FEETECH USB adapter is plugged in and that this shell can see the device." >&2
    else
        echo "[ERROR] Multiple serial candidates were detected, so the script cannot safely guess which one to use:" >&2
        printf '  %s\n' "${serial_candidates[@]}" >&2
        echo "[ERROR] Either fix the udev symlink or point --teach-config at a YAML file with the right port." >&2
    fi
    return 1
}

wait_for_joint_states() {
    local timeout_sec="$1"
    local deadline=$((SECONDS + timeout_sec))

    while (( SECONDS < deadline )); do
        if [[ -n "${TEACH_STATE_PUBLISHER_PID}" ]] && ! kill -0 "${TEACH_STATE_PUBLISHER_PID}" 2>/dev/null; then
            echo "[ERROR] ftservo_hls3625_teach bus_state_publisher exited before publishing /joint_states" >&2
            return 1
        fi

        if ros2 topic echo /joint_states sensor_msgs/msg/JointState --once >/dev/null 2>&1; then
            return 0
        fi

        sleep 0.2
    done

    return 1
}

wait_for_remote_ready_true() {
    local timeout_sec="$1"
    local deadline=$((SECONDS + timeout_sec))

    while (( SECONDS < deadline )); do
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
        --teach-config)
            TEACH_CONFIG_FILE="$2"
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
    echo "        Build the workspace first, e.g. colcon build --base-paths src --packages-select arm2_task ftservo_hls3625_teach" >&2
    exit 1
fi

if [[ ! -f "${ARM_PARAMS_FILE}" ]]; then
    echo "[ERROR] arm2_task params file not found: ${ARM_PARAMS_FILE}" >&2
    exit 1
fi

if [[ ! -f "${TEACH_CONFIG_FILE}" ]]; then
    echo "[ERROR] ftservo_hls3625_teach config file not found: ${TEACH_CONFIG_FILE}" >&2
    exit 1
fi

trap cleanup EXIT INT TERM

cd "${SCRIPT_DIR}"

source_setup "${ROS_SETUP}"
source_setup "${WS_SETUP}"
record_existing_ros2_daemons

assert_binary_runnable "${SCRIPT_DIR}/install/ftservo_hls3625_teach/lib/ftservo_hls3625_teach/bus_state_publisher" "bus_state_publisher"
assert_binary_runnable "${SCRIPT_DIR}/install/arm2_task/lib/arm2_task/teach_pendant_follow_node" "teach_pendant_follow_node"

echo "[INFO] arm2_task params: ${ARM_PARAMS_FILE}"
echo "[INFO] teach config: ${TEACH_CONFIG_FILE}"

prepare_teach_config
echo "[INFO] resolved teach config: ${TEACH_CONFIG_FILE}"

echo "[INFO] Launching ftservo_hls3625_teach state publisher..."
launch_in_group TEACH_STATE_PUBLISHER_PID ros2 launch ftservo_hls3625_teach state_publisher.launch.py config:="${TEACH_CONFIG_FILE}"

echo "[INFO] Waiting for /joint_states (timeout: ${READY_TIMEOUT}s)..."
if ! wait_for_joint_states "${READY_TIMEOUT}"; then
    echo "[ERROR] Timed out waiting for /joint_states" >&2
    exit 1
fi

echo "[INFO] Waiting for remote /robot_driver/ready == true (timeout: ${READY_TIMEOUT}s)..."
if ! wait_for_remote_ready_true "${READY_TIMEOUT}"; then
    echo "[ERROR] Timed out waiting for remote /robot_driver/ready == true" >&2
    exit 1
fi

echo "[INFO] Launching teach_pendant_follow_node..."
launch_in_group TEACH_BRIDGE_PID ros2 launch arm2_task teach_pendant_follow_launch.py params_path:="${ARM_PARAMS_FILE}"

echo "[INFO] Pendant-side teach pendant stack is ready."

wait -n "${TEACH_STATE_PUBLISHER_PID}" "${TEACH_BRIDGE_PID}"
EXIT_CODE=$?
echo "[ERROR] One of the pendant-side teach pendant processes exited unexpectedly" >&2
exit "${EXIT_CODE}"
