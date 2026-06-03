#!/usr/bin/env bash
set -euo pipefail

ALIAS_NAME="${1:-esp32_suction_c3}"
DEVICE_PATH="${2:-}"
RULE_FILE="/etc/udev/rules.d/99-${ALIAS_NAME}.rules"

usage() {
  cat <<EOF
Usage:
  sudo ./scripts/install_esp32_suction_udev.sh [alias_name] [device_path]

Examples:
  sudo ./scripts/install_esp32_suction_udev.sh
  sudo ./scripts/install_esp32_suction_udev.sh esp32_suction_c3 /dev/ttyACM0

Behavior:
  - Detects the attached Espressif USB JTAG/serial debug unit
  - Extracts its vendor, product, and serial attributes
  - Installs a persistent udev symlink rule at:
      ${RULE_FILE}
  - Reloads udev rules and triggers the tty device

Result:
  The board will appear as /dev/<alias_name>
EOF
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

detect_device() {
  local candidate

  if [[ -n "${DEVICE_PATH}" ]]; then
    if [[ ! -e "${DEVICE_PATH}" ]]; then
      echo "Specified device does not exist: ${DEVICE_PATH}" >&2
      exit 1
    fi
    echo "${DEVICE_PATH}"
    return
  fi

  while IFS= read -r candidate; do
    echo "${candidate}"
    return
  done < <(find /dev/serial/by-id -maxdepth 1 -type l \( -name '*Espressif*' -o -name '*esp32*' -o -name '*ESP32*' \) 2>/dev/null | sort)

  while IFS= read -r candidate; do
    if udevadm info -a -n "${candidate}" 2>/dev/null | grep -q 'ATTRS{manufacturer}=="Espressif"'; then
      echo "${candidate}"
      return
    fi
  done < <(find /dev/serial/by-id -maxdepth 1 -type l -name 'usb-*' 2>/dev/null | sort)

  while IFS= read -r candidate; do
    if udevadm info -a -n "${candidate}" 2>/dev/null | grep -q 'ATTRS{manufacturer}=="Espressif"'; then
      echo "${candidate}"
      return
    fi
  done < <(find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) | sort)

  echo "No Espressif USB serial device found." >&2
  exit 1
}

resolve_device_for_udevadm() {
  local device="$1"

  if [[ -L "${device}" ]]; then
    readlink -f "${device}"
  else
    echo "${device}"
  fi
}

extract_attr() {
  local key="$1"
  local info="$2"
  local value

  value="$(printf '%s\n' "${info}" | sed -n "s/.*ATTRS{${key}}==\"\\([^\"]*\\)\"/\\1/p" | head -n1)"
  if [[ -z "${value}" ]]; then
    echo "Failed to extract udev attribute: ${key}" >&2
    exit 1
  fi
  printf '%s\n' "${value}"
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi

  require_cmd udevadm
  require_cmd sed
  require_cmd grep
  require_cmd find

  if [[ "${EUID}" -ne 0 ]]; then
    echo "Please run this script with sudo." >&2
    exit 1
  fi

  local device udev_device info vendor product serial escaped_serial
  device="$(detect_device)"
  udev_device="$(resolve_device_for_udevadm "${device}")"
  info="$(udevadm info -a -n "${udev_device}")"

  vendor="$(extract_attr idVendor "${info}")"
  product="$(extract_attr idProduct "${info}")"
  serial="$(extract_attr serial "${info}")"
  escaped_serial="$(printf '%s' "${serial}" | sed 's/"/\\"/g')"

  cat > "${RULE_FILE}" <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="${vendor}", ATTRS{idProduct}=="${product}", ATTRS{serial}=="${escaped_serial}", SYMLINK+="${ALIAS_NAME}", MODE="0666"
EOF

  udevadm control --reload-rules
  udevadm trigger --action=add "${device}"
  udevadm settle

  echo "Installed udev rule: ${RULE_FILE}"
  echo "Matched device: ${device}"
  echo "Resolved device: ${udev_device}"
  echo "Alias path: /dev/${ALIAS_NAME}"
  if [[ -e "/dev/${ALIAS_NAME}" ]]; then
    ls -l "/dev/${ALIAS_NAME}"
  else
    echo "Alias will appear after the device is replugged if it is not visible yet."
  fi
}

main "$@"
