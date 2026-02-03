#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="/opt/ros/noetic/setup.bash"
PKG="ft_sensor_ros"
LAUNCH="sensorFT.launch"

FRAME_ID_DEFAULT="ft_sensor"
TOPIC_DEFAULT="/ft_sensor/wrench"

DISABLE_OFFLOAD_DEFAULT=1

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--iface IFACE] [--rate HZ] [--frame FRAME] [--topic TOPIC]
                   [--tare-samples N] [--tare-period-ms MS]
                   [--warmup S] [--stability-samples N] [--stability-period-ms MS]
                   [--force-std N] [--torque-std Nm] [--max-wait S]
                   [--k-sigma K] [--no-offload]

Examples:
  $(basename "$0") --iface enx207bd51ab7ad
  $(basename "$0") --rate 100 --tare-samples 500
  $(basename "$0")   # auto-detect iface (first "enx*")
EOF
}

# ---------------------------
# Workspace auto-detection
# ---------------------------
find_ws_setup() {
  # 1) Try fast path: script is at <WS>/src/<pkg>/scripts -> go up 3 levels
  local script_dir ws_root
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  ws_root="$(cd "${script_dir}/../../.." && pwd)"

  if [[ -f "${ws_root}/devel/setup.bash" ]]; then
    echo "${ws_root}/devel/setup.bash"
    return 0
  fi
  if [[ -f "${ws_root}/install/setup.bash" ]]; then
    echo "${ws_root}/install/setup.bash"
    return 0
  fi

  # 2) Robust fallback: walk up from script_dir until we find devel/ or install/
  local cur="${script_dir}"
  while [[ "${cur}" != "/" ]]; do
    if [[ -f "${cur}/devel/setup.bash" ]]; then
      echo "${cur}/devel/setup.bash"
      return 0
    fi
    if [[ -f "${cur}/install/setup.bash" ]]; then
      echo "${cur}/install/setup.bash"
      return 0
    fi
    cur="$(dirname "${cur}")"
  done

  return 1
}

# ---------------------------
# Parse args
# ---------------------------
IFACE=""
RATE_HZ="50"
FRAME_ID="${FRAME_ID_DEFAULT}"
TOPIC="${TOPIC_DEFAULT}"

TARE_SAMPLES="300"
TARE_PERIOD_MS="20"

WARMUP_S="15"
STAB_SAMPLES="50"
STAB_PERIOD_MS="20"
FORCE_STD="0.05"
TORQUE_STD="0.005"
MAX_WAIT="60.0"
K_SIGMA="3.0"

DISABLE_OFFLOAD="${DISABLE_OFFLOAD_DEFAULT}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --iface) IFACE="${2:-}"; shift 2 ;;
    --rate) RATE_HZ="${2:-}"; shift 2 ;;
    --frame) FRAME_ID="${2:-}"; shift 2 ;;
    --topic) TOPIC="${2:-}"; shift 2 ;;

    --tare-samples) TARE_SAMPLES="${2:-}"; shift 2 ;;
    --tare-period-ms) TARE_PERIOD_MS="${2:-}"; shift 2 ;;

    --warmup) WARMUP_S="${2:-}"; shift 2 ;;
    --stability-samples) STAB_SAMPLES="${2:-}"; shift 2 ;;
    --stability-period-ms) STAB_PERIOD_MS="${2:-}"; shift 2 ;;
    --force-std) FORCE_STD="${2:-}"; shift 2 ;;
    --torque-std) TORQUE_STD="${2:-}"; shift 2 ;;
    --max-wait) MAX_WAIT="${2:-}"; shift 2 ;;
    --k-sigma) K_SIGMA="${2:-}"; shift 2 ;;

    --no-offload) DISABLE_OFFLOAD="0"; shift 1 ;;
    *)
      echo "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

# ---------------------------
# Auto-detect iface if needed
# ---------------------------
if [[ -z "${IFACE}" ]]; then
  IFACE="$(ip -br link | awk '$1 ~ /^enx/ {print $1; exit}')"
  if [[ -z "${IFACE}" ]]; then
    echo "ERROR: --iface not provided and no 'enx*' interface found."
    echo "Run: ip -br link"
    exit 1
  fi
  echo "Auto-detected iface: ${IFACE}"
fi

# ---------------------------
# Sanity checks
# ---------------------------
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found: ${ROS_SETUP}"
  exit 1
fi

WS_SETUP="$(find_ws_setup || true)"
if [[ -z "${WS_SETUP}" ]]; then
  echo "ERROR: Could not locate a catkin workspace setup.bash."
  echo "Make sure you built the workspace (catkin_make) so devel/setup.bash exists."
  exit 1
fi

echo "Using workspace overlay: ${WS_SETUP}"

# ---------------------------
# Optional: disable NIC offloads (recommended)
# ---------------------------
if [[ "${DISABLE_OFFLOAD}" == "1" ]]; then
  echo "Disabling NIC offloads on ${IFACE} (recommended for EtherCAT)..."
  sudo ethtool -K "${IFACE}" gro off gso off tso off sg off >/dev/null 2>&1 || true
fi

echo "Launching ${PKG}/${LAUNCH}"
echo "  iface=${IFACE}"
echo "  frame_id=${FRAME_ID}"
echo "  topic=${TOPIC}"
echo "  rate_hz=${RATE_HZ}"
echo "  tare_samples=${TARE_SAMPLES} tare_period_ms=${TARE_PERIOD_MS}"
echo "  warmup_seconds=${WARMUP_S} stability_samples=${STAB_SAMPLES} stability_period_ms=${STAB_PERIOD_MS}"
echo "  stability_force_std_thresh_N=${FORCE_STD} stability_torque_std_thresh_Nm=${TORQUE_STD} stability_max_wait_s=${MAX_WAIT}"
echo "  tare_outlier_k_sigma=${K_SIGMA}"

# ---------------------------
# Run roslaunch as root in an isolated env
# ---------------------------
sudo bash -lc "
  source '${ROS_SETUP}'
  source '${WS_SETUP}'
  exec roslaunch '${PKG}' '${LAUNCH}' \
    iface:='${IFACE}' frame_id:='${FRAME_ID}' topic:='${TOPIC}' rate_hz:='${RATE_HZ}' \
    tare_samples:='${TARE_SAMPLES}' tare_period_ms:='${TARE_PERIOD_MS}' \
    warmup_seconds:='${WARMUP_S}' stability_samples:='${STAB_SAMPLES}' stability_period_ms:='${STAB_PERIOD_MS}' \
    stability_force_std_thresh_N:='${FORCE_STD}' stability_torque_std_thresh_Nm:='${TORQUE_STD}' stability_max_wait_s:='${MAX_WAIT}' \
    tare_outlier_k_sigma:='${K_SIGMA}'
"

