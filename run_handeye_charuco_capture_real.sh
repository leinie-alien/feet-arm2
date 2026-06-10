#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$SCRIPT_DIR/install/setup.bash"
DEFAULT_ARM_PARAMS_FILE="$SCRIPT_DIR/src/arm2_task/config/params.yaml"
DEFAULT_DRIVER_PARAMS_FILE="$SCRIPT_DIR/src/dm_motor_sdk_ros/config/dm_motor_robot_driver.yaml"
DEFAULT_INTRINSICS_EXPORT="$SCRIPT_DIR/recordings/handeye_charuco_capture/device_intrinsics.yml"

ARM_PARAMS_FILE="$DEFAULT_ARM_PARAMS_FILE"
DRIVER_PARAMS_FILE="$DEFAULT_DRIVER_PARAMS_FILE"
OUTPUT_DIR=""
INTRINSICS_FILE=""
CAMERA_SOURCE=""
READY_TIMEOUT=10
READ_INTRINSICS_FROM_DEVICE=0
INTRINSICS_WIDTH=1280
INTRINSICS_HEIGHT=720
INTRINSICS_FPS=30
DRIVER_PID=""
GRAVITY_PID=""
CAPTURE_PID=""

usage() {
    cat <<USAGE
Usage: $(basename "$0") [options] [-- <extra ros args>]

Handeye ChArUco capture startup:
1. Launch dm_motor_sdk_ros driver
2. Wait for /robot_driver/ready == true
3. Launch gravity_comp_test_node in gravity_comp mode
4. Launch handeye_charuco_capture_node in an xterm for keyboard-triggered sampling

Options:
  --arm-params <file>      arm2_task parameter file
  --driver-params <file>   dm_motor_sdk_ros driver parameter file
  --output-dir <dir>       capture output directory
  --intrinsics-file <file> OpenCV YAML/XML with camera_matrix and dist_coeffs
  --read-intrinsics-from-device
                           export intrinsics from the connected RealSense first
  --camera-source <src>    OpenCV camera source, e.g. 0 or /dev/video4
  --intrinsics-width <px>  color stream width used when exporting intrinsics
  --intrinsics-height <px> color stream height used when exporting intrinsics
  --intrinsics-fps <hz>    color stream fps used when exporting intrinsics
  --ready-timeout <sec>    wait timeout for /robot_driver/ready (default: ${READY_TIMEOUT})
  -h, --help               show this help

Anything after '--' is forwarded to handeye_charuco_capture_node.
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
    stop_process_group "${CAPTURE_PID}" "handeye_charuco_capture_node xterm"
    stop_process_group "${GRAVITY_PID}" "gravity_comp_test_node"
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

EXTRA_ROS_ARGS=()
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
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --intrinsics-file)
            INTRINSICS_FILE="$2"
            shift 2
            ;;
        --read-intrinsics-from-device)
            READ_INTRINSICS_FROM_DEVICE=1
            shift
            ;;
        --camera-source)
            CAMERA_SOURCE="$2"
            shift 2
            ;;
        --intrinsics-width)
            INTRINSICS_WIDTH="$2"
            shift 2
            ;;
        --intrinsics-height)
            INTRINSICS_HEIGHT="$2"
            shift 2
            ;;
        --intrinsics-fps)
            INTRINSICS_FPS="$2"
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

if ! command -v xterm >/dev/null 2>&1; then
    echo "[ERROR] xterm is not installed or not in PATH." >&2
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

if [[ -n "${INTRINSICS_FILE}" && ! -f "${INTRINSICS_FILE}" ]]; then
    echo "[ERROR] intrinsics file not found: ${INTRINSICS_FILE}" >&2
    exit 1
fi

trap cleanup EXIT INT TERM

cd "${SCRIPT_DIR}"
source_setup "${ROS_SETUP}"
source_setup "${WS_SETUP}"
export ARM_WORKSPACE_DIR="${SCRIPT_DIR}"

if [[ "${READ_INTRINSICS_FROM_DEVICE}" == "1" ]]; then
    INTRINSICS_FILE="${DEFAULT_INTRINSICS_EXPORT}"
    echo "[INFO] Exporting RealSense intrinsics to ${INTRINSICS_FILE} ..."
    python3 "${SCRIPT_DIR}/tools/export_realsense_intrinsics.py" \
        --output "${INTRINSICS_FILE}" \
        --width "${INTRINSICS_WIDTH}" \
        --height "${INTRINSICS_HEIGHT}" \
        --fps "${INTRINSICS_FPS}"
fi

if [[ -z "${INTRINSICS_FILE}" ]]; then
    echo "[ERROR] Missing intrinsics source. Provide --intrinsics-file or --read-intrinsics-from-device." >&2
    exit 1
fi

echo "[INFO] Launching dm_motor_sdk_ros driver..."
launch_in_group DRIVER_PID ros2 launch dm_motor_sdk_ros dm_motor_robot_driver.launch.py params_path:="${DRIVER_PARAMS_FILE}"

echo "[INFO] Waiting for /robot_driver/ready ..."
if ! wait_for_ready_true "${READY_TIMEOUT}"; then
    echo "[ERROR] Timed out waiting for /robot_driver/ready == true" >&2
    exit 1
fi

echo "[INFO] Launching gravity_comp_test_node in gravity_comp mode..."
launch_in_group GRAVITY_PID ros2 run arm2_task gravity_comp_test_node --ros-args --params-file "${ARM_PARAMS_FILE}" -p gravity_comp_test.gains_mode:=gravity_comp

CAPTURE_CMD=(ros2 run arm2_task handeye_charuco_capture_node --ros-args --params-file "${ARM_PARAMS_FILE}")
CAPTURE_CMD+=(-p "handeye_charuco_capture.require_ready:=false")
if [[ -n "${OUTPUT_DIR}" ]]; then
    CAPTURE_CMD+=(-p "handeye_charuco_capture.output_dir:=${OUTPUT_DIR}")
fi
if [[ -n "${INTRINSICS_FILE}" ]]; then
    CAPTURE_CMD+=(-p "handeye_charuco_capture.intrinsics_file:=${INTRINSICS_FILE}")
fi
if [[ -n "${CAMERA_SOURCE}" ]]; then
    CAPTURE_CMD+=(-p "handeye_charuco_capture.camera_source:='${CAMERA_SOURCE}'")
fi
if [[ ${#EXTRA_ROS_ARGS[@]} -gt 0 ]]; then
    CAPTURE_CMD+=("${EXTRA_ROS_ARGS[@]}")
fi

CAPTURE_SHELL_CMD="$(printf '%q ' "${CAPTURE_CMD[@]}")"

echo "[INFO] Launching handeye_charuco_capture_node in xterm..."
echo "[INFO] Keyboard in xterm: c=capture, v=preview, u=undo, s=save, h=help, q=quit"
launch_in_group CAPTURE_PID xterm -T "Handeye ChArUco Capture" -e bash -lc "${CAPTURE_SHELL_CMD}; status=\$?; echo; echo '[INFO] Capture session ended. Press Enter to close this window.'; read -r _; exit \$status"

wait "${CAPTURE_PID}"
