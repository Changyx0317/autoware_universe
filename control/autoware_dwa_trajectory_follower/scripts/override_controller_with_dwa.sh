#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NS="/control/control_container"
DWA_PARAM_FILE="/home/chang/autoware/install/autoware_launch/share/autoware_launch/config/control/trajectory_follower/dwa_trajectory_follower.param.yaml"
VEHICLE_PARAM_FILE="/home/chang/autoware/install/sample_vehicle_description/share/sample_vehicle_description/config/vehicle_info.param.yaml"
WAIT_SEC=20
SKIP_UNLOAD=0
AUTO_ENGAGE=1
DIRECT_PASSTHROUGH=0
RELAY_PID=""
GEAR_RELAY_PID=""

usage() {
  cat <<'EOF'
Usage:
  override_controller_with_dwa.sh [options]

Options:
  --container <ns>         Component container namespace (default: /control/control_container)
  --dwa-param <file>       DWA parameter yaml file
  --vehicle-param <file>   Vehicle info parameter yaml file
  --wait-sec <sec>         Wait seconds for control container (default: 20)
  --skip-unload            Do not unload built-in controller, only launch DWA
  --no-auto-engage         Do not call /api/autoware/set/engage automatically
  --auto-engage            Force call /api/autoware/set/engage automatically (default)
  --direct-passthrough     Simulation passthrough: unload vehicle_cmd_gate and relay
                           /control/trajectory_follower/control_cmd -> /control/command/control_cmd
                           Also relay /control/shift_decider/gear_cmd -> /control/command/gear_cmd
  -h, --help               Show this help

Example:
  ros2 run autoware_dwa_trajectory_follower override_controller_with_dwa.sh \
    --dwa-param /home/chang/autoware/install/autoware_launch/share/autoware_launch/config/control/trajectory_follower/dwa_trajectory_follower.param.yaml \
    --vehicle-param /home/chang/autoware/install/sample_vehicle_description/share/sample_vehicle_description/config/vehicle_info.param.yaml
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --container)
      CONTAINER_NS="${2:-}"; shift 2 ;;
    --dwa-param)
      DWA_PARAM_FILE="${2:-}"; shift 2 ;;
    --vehicle-param)
      VEHICLE_PARAM_FILE="${2:-}"; shift 2 ;;
    --wait-sec)
      WAIT_SEC="${2:-}"; shift 2 ;;
    --skip-unload)
      SKIP_UNLOAD=1; shift ;;
    --no-auto-engage)
      AUTO_ENGAGE=0; shift ;;
    --auto-engage)
      AUTO_ENGAGE=1; shift ;;
    --direct-passthrough)
      DIRECT_PASSTHROUGH=1; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "[DWA override] Unknown option: $1" >&2
      usage
      exit 2 ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[DWA override] ros2 command not found. Please source ROS/Autoware setup first." >&2
  exit 1
fi

if [[ ! -f "${DWA_PARAM_FILE}" ]]; then
  echo "[DWA override] DWA param file not found: ${DWA_PARAM_FILE}" >&2
  exit 1
fi

if [[ ! -f "${VEHICLE_PARAM_FILE}" ]]; then
  echo "[DWA override] Vehicle param file not found: ${VEHICLE_PARAM_FILE}" >&2
  exit 1
fi

maybe_auto_engage() {
  [[ "${DIRECT_PASSTHROUGH}" -eq 0 ]] || return 0
  [[ "${AUTO_ENGAGE}" -eq 1 ]] || return 0

  local service_name="/api/autoware/set/engage"
  local service_type=""

  if ! service_type="$(ros2 service type "${service_name}" 2>/dev/null)"; then
    echo "[DWA override] Engage service not available yet: ${service_name} (skip auto-engage)"
    return 0
  fi

  if [[ -z "${service_type}" ]]; then
    echo "[DWA override] Engage service type is empty (skip auto-engage)"
    return 0
  fi

  echo "[DWA override] Auto-engage via ${service_name} (${service_type})"
  if ros2 service call "${service_name}" "${service_type}" "{engage: true}" >/dev/null 2>&1; then
    echo "[DWA override] Auto-engage request sent."
  else
    echo "[DWA override] Auto-engage call failed; continue anyway."
  fi
}

unload_component_by_suffix() {
  local suffix="$1"
  local ids=""
  ids="$(ros2 component list "${CONTAINER_NS}" 2>/dev/null | awk -v s="${suffix}" '$2 ~ s"$" {print $1}')"
  if [[ -n "${ids}" ]]; then
    while read -r id; do
      [[ -z "${id}" ]] && continue
      echo "[DWA override] Unloading component id=${id} (${suffix}) from ${CONTAINER_NS}"
      ros2 component unload "${CONTAINER_NS}" "${id}"
    done <<< "${ids}"
  fi
}

start_direct_passthrough_relays() {
  [[ "${DIRECT_PASSTHROUGH}" -eq 1 ]] || return 0

  echo "[DWA override] DIRECT PASSTHROUGH enabled (simulation only)."
  unload_component_by_suffix "/control/vehicle_cmd_gate"

  echo "[DWA override] Relay: /control/trajectory_follower/control_cmd -> /control/command/control_cmd"
  ros2 run topic_tools relay /control/trajectory_follower/control_cmd /control/command/control_cmd >/tmp/dwa_passthrough_relay.log 2>&1 &
  RELAY_PID="$!"

  echo "[DWA override] Relay: /control/shift_decider/gear_cmd -> /control/command/gear_cmd"
  ros2 run topic_tools relay /control/shift_decider/gear_cmd /control/command/gear_cmd >/tmp/dwa_passthrough_gear_relay.log 2>&1 &
  GEAR_RELAY_PID="$!"
}

cleanup_relays() {
  if [[ -n "${RELAY_PID}" ]] && kill -0 "${RELAY_PID}" 2>/dev/null; then
    kill "${RELAY_PID}" 2>/dev/null || true
  fi
  if [[ -n "${GEAR_RELAY_PID}" ]] && kill -0 "${GEAR_RELAY_PID}" 2>/dev/null; then
    kill "${GEAR_RELAY_PID}" 2>/dev/null || true
  fi
}

if [[ "${SKIP_UNLOAD}" -eq 0 ]]; then
  echo "[DWA override] Waiting for ${CONTAINER_NS} ..."
  found=0
  for ((i=0; i<WAIT_SEC; ++i)); do
    if ros2 node list 2>/dev/null | grep -Fxq "${CONTAINER_NS}"; then
      found=1
      break
    fi
    sleep 1
  done

  if [[ "${found}" -eq 0 ]]; then
    echo "[DWA override] Container not found: ${CONTAINER_NS}" >&2
    echo "[DWA override] Start Autoware first, or use --skip-unload if you know what you're doing." >&2
    exit 1
  fi

  echo "[DWA override] Checking built-in controller components ..."
  ids="$(ros2 component list "${CONTAINER_NS}" 2>/dev/null | awk '$2 ~ /controller_node_exe$/ {print $1}')"

  if [[ -n "${ids}" ]]; then
    while read -r id; do
      [[ -z "${id}" ]] && continue
      echo "[DWA override] Unloading controller component id=${id} from ${CONTAINER_NS}"
      ros2 component unload "${CONTAINER_NS}" "${id}"
    done <<< "${ids}"
  else
    echo "[DWA override] No controller_node_exe component found in ${CONTAINER_NS} (already unloaded or not component mode)."
  fi
fi

start_direct_passthrough_relays
maybe_auto_engage

echo "[DWA override] Launching external DWA node ..."
trap cleanup_relays EXIT
ros2 run autoware_dwa_trajectory_follower dwa_trajectory_follower_node \
  --ros-args \
  -r __node:=controller_node_exe \
  -r __ns:=/control/trajectory_follower \
  --params-file "${DWA_PARAM_FILE}" \
  --params-file "${VEHICLE_PARAM_FILE}"
