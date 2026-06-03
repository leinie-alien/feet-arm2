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
SERVICE_TIMEOUT=10
PAYLOAD_MASS=0.5
PAYLOAD_COM="0.0,0.0,0.2219"
PAYLOAD_SERVICE_PARAM="set_payload_state"
PAYLOAD_SERVICE_CALL="/set_payload_state"

DRIVER_PID=""
APP_PID=""
CLEANUP_RUNNING=0
ROS2_DAEMON_PATTERN='[r]os2cli\.daemon\.daemonize.*--name ros2-daemon'
declare -a ROS2_DAEMON_PIDS_BEFORE=()

usage() {
    cat <<USAGE
Usage: $(basename "$0") [options] [-- <extra ros args>]

One-click startup for real robot gravity compensation with payload enabled:
1. Launch dm_motor_sdk_ros driver
2. Wait for /robot_driver/ready == true
3. Run arm2_task gravity_comp_test_node with payload service enabled
4. Wait for the payload service to appear
5. Call SetPayloadState to enable the payload model

Options:
  --params <file>           arm2_task parameter file
  --driver-params <file>    dm_motor_sdk_ros driver parameter file
  --ready-timeout <sec>     wait timeout for /robot_driver/ready (default: ${READY_TIMEOUT})
  --service-timeout <sec>   wait timeout for payload service (default: ${SERVICE_TIMEOUT})
  --mass <kg>               payload mass in kg (default: ${PAYLOAD_MASS})
  --com <x,y,z>             payload COM in meters (default: ${PAYLOAD_COM})
  --payload-service <name>  payload service name (default: ${PAYLOAD_SERVICE_PARAM})
  -h, --help                show this help

Anything after '--' is forwarded to gravity_comp_test_node.
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
        "${APP_PID}"
        "${DRIVER_PID}"
    )
    local names=(
        "gravity_comp_test_node"
        "dm_motor_sdk_ros driver"
    )

    for i in "${!pids[@]}"; do
        stop_process_group "${pids[$i]}" "${names[$i]}"
    done

    stop_new_ros2_daemons
}

normalize_ros_name() {
    local name="$1"
    if [[ "${name}" == /* ]]; then
        printf '%s' "${name}"
    else
        printf '/%s' "${name}"
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

wait_for_service_available() {
    local service_name="$1"
    local timeout_sec="$2"
    local deadline=$((SECONDS + timeout_sec))

    while (( SECONDS < deadline )); do
        if [[ -n "${APP_PID}" ]] && ! kill -0 "${APP_PID}" 2>/dev/null; then
            echo "[ERROR] gravity_comp_test_node exited before service ${service_name} became available" >&2
            return 1
        fi

        if ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
            return 0
        fi

        sleep 0.2
    done

    return 1
}

call_payload_service() {
    local service_name="$1"
    local mass="$2"
    local com_x="$3"
    local com_y="$4"
    local com_z="$5"

    local output=""
    if ! output=$(ros2 service call "${service_name}" robot_msgs/srv/SetPayloadState \
        "{has_load: true, mass: ${mass}, com: [${com_x}, ${com_y}, ${com_z}]}" 2>&1); then
        echo "[ERROR] Failed to call ${service_name}" >&2
        echo "${output}" >&2
        return 1
    fi

    echo "${output}"
    if ! grep -Eqi "success:[[:space:]]*true|success=True" <<< "${output}"; then
        echo "[ERROR] Payload service responded without success=true" >&2
        return 1
    fi

    return 0
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
        --service-timeout)
            SERVICE_TIMEOUT="$2"
            shift 2
            ;;
        --mass)
            PAYLOAD_MASS="$2"
            shift 2
            ;;
        --com)
            PAYLOAD_COM="$2"
            shift 2
            ;;
        --payload-service)
            PAYLOAD_SERVICE_PARAM="$2"
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
    echo "        Build the workspace first, e.g. colcon build --packages-select dm_motor_sdk_ros robot_msgs arm2_task" >&2
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

IFS=',' read -r PAYLOAD_COM_X PAYLOAD_COM_Y PAYLOAD_COM_Z <<< "${PAYLOAD_COM}"
if [[ -z "${PAYLOAD_COM_X:-}" || -z "${PAYLOAD_COM_Y:-}" || -z "${PAYLOAD_COM_Z:-}" ]]; then
    echo "[ERROR] Invalid --com format. Expected x,y,z but got: ${PAYLOAD_COM}" >&2
    exit 1
fi

PAYLOAD_SERVICE_CALL="$(normalize_ros_name "${PAYLOAD_SERVICE_PARAM}")"

trap cleanup EXIT INT TERM

cd "${SCRIPT_DIR}"

source_setup "${ROS_SETUP}"
source_setup "${WS_SETUP}"
record_existing_ros2_daemons

echo "[INFO] arm2_task params: ${PARAMS_FILE}"
echo "[INFO] driver params: ${DRIVER_PARAMS_FILE}"
echo "[INFO] payload service: ${PAYLOAD_SERVICE_CALL}"
echo "[INFO] payload mass: ${PAYLOAD_MASS} kg"
echo "[INFO] payload COM: [${PAYLOAD_COM_X}, ${PAYLOAD_COM_Y}, ${PAYLOAD_COM_Z}]"

echo "[INFO] Launching dm_motor_sdk_ros driver..."
launch_in_group DRIVER_PID ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py params_path:="${DRIVER_PARAMS_FILE}"

echo "[INFO] Waiting for /robot_driver/ready == true (timeout: ${READY_TIMEOUT}s)..."
if ! wait_for_ready_true "${READY_TIMEOUT}"; then
    echo "[ERROR] Timed out waiting for /robot_driver/ready == true" >&2
    exit 1
fi

echo "[INFO] Starting gravity_comp_test_node with payload service enabled..."
launch_in_group APP_PID ros2 run arm2_task gravity_comp_test_node \
    --ros-args \
    --params-file "${PARAMS_FILE}" \
    -p "gravity_comp_test.enable_payload_service:=true" \
    -p "gravity_comp_test.gains_mode:=loaded" \
    -p "gravity_comp_test.payload_service:=${PAYLOAD_SERVICE_PARAM}" \
    "${EXTRA_ROS_ARGS[@]}"

echo "[INFO] Waiting for payload service ${PAYLOAD_SERVICE_CALL} (timeout: ${SERVICE_TIMEOUT}s)..."
if ! wait_for_service_available "${PAYLOAD_SERVICE_CALL}" "${SERVICE_TIMEOUT}"; then
    echo "[ERROR] Timed out waiting for payload service ${PAYLOAD_SERVICE_CALL}" >&2
    exit 1
fi

echo "[INFO] Enabling payload model..."
if ! call_payload_service "${PAYLOAD_SERVICE_CALL}" "${PAYLOAD_MASS}" "${PAYLOAD_COM_X}" "${PAYLOAD_COM_Y}" "${PAYLOAD_COM_Z}"; then
    exit 1
fi

echo "[INFO] Gravity compensation loaded startup is ready."
wait "${APP_PID}"
APP_RC=$?
APP_PID=""
exit "${APP_RC}"
