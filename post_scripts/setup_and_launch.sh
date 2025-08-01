#!/bin/bash
set -euo pipefail

# Source common function libraries
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"
source "${SCRIPT_DIR}/common_git.sh"
source "${SCRIPT_DIR}/common_ssh.sh"
source "${SCRIPT_DIR}/common_build.sh"

# === Configurable Mappings ===
declare -A PI_USERS=(
  [127.0.0.1]="rospi"
  [172.23.254.18]="rospi"
  [172.23.254.22]="rospi"
  [172.23.254.23]="rospi"
  [172.23.254.24]="rospi"
)

declare -A PI_NAMES=(
  [127.0.0.1]="default_graveyard"
  [172.23.254.24]="rospi_1"
  [172.23.254.22]="rospi_2"
  [172.23.254.23]="rospi_3"
  [172.23.254.18]="rospi_4"
)

declare -A PI_TYPES=(
  [127.0.0.1]="graveyard"
  [172.23.254.18]="default"
  [172.23.254.22]="default"
  [172.23.254.23]="default"
  [172.23.254.24]="default"
)

# === Launch using local tmux session ===
launch_station_tmux_local() {
  local ip="$1"
  local user="$2"
  local node_name="$3"
  local node_type="$4"
  local ws_path="$5"
  local dds_config="$6"
  local qos_profile="$7"
  local qos_depth="$8"

  local session_name="post_launch"
  local window_name="${ip//./_}"

  # Create new window (or session if none exists)
  if ! tmux has-session -t "$session_name" 2>/dev/null; then
    tmux new-session -d -s "$session_name" -n "$window_name"
  else
    if tmux list-windows -t "$session_name" | grep -q "^$window_name"; then
      tmux kill-window -t "${session_name}:$window_name"
    fi
    tmux new-window -t "$session_name" -n "$window_name"
  fi

  local remote_cmd="cd \"${ws_path}\""
  remote_cmd+=" && chmod -R +rwx ."
  remote_cmd+=" && source \"${ws_path}/src/post/post_scripts/${dds_config}\" \"${ws_path}\""
  remote_cmd+=" && set +u"
  remote_cmd+=" && source \"${ws_path}/install/setup.bash\""
  #remote_cmd+=" && pkill -f post_core"
  remote_cmd+=" && ros2 run post_core station"
  remote_cmd+=" --name ${node_name}"
  remote_cmd+=" --type ${node_type}"
  remote_cmd+=" --lossmode ${qos_profile}"
  remote_cmd+=" --depth ${qos_depth}"

  tmux send-keys -t "${session_name}:${window_name}" \
    "ssh -o StrictHostKeyChecking=no ${user}@${ip} bash -c '$remote_cmd'" C-m
}

# === YAD Form ===
DEFAULT_WORKSPACE="${HOME}/Desktop/test_ws"
if [[ -d "${DEFAULT_WORKSPACE}/src/post" ]]; then
  cd "${DEFAULT_WORKSPACE}/src/post"
  DEFAULT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "main")
  cd - > /dev/null
else
  DEFAULT_BRANCH="main"
fi
DEFAULT_DDS="cyclonedds_source.sh"
DEFAULT_QOS_PROFILE="lossless"
DEFAULT_QOS_DEPTH="10"

FORM_OUTPUT=$(yad --form --title="Launch Parcel Script" \
  --text="Configure Launch Parameters" \
  --field="Workspace Folder:TXT" "${DEFAULT_WORKSPACE}" \
  --field="Branch Name:TXT" "${DEFAULT_BRANCH}" \
  --field="DDS Config:CB" "cyclonedds_source.sh!fastrtps_source.sh" \
  --field="QOS Profile:CB" "lossless!lossy" \
  --field="QOS Depth:NUM" "${DEFAULT_QOS_DEPTH}" \
  --field="Pull from GitHub:CHK" "TRUE" \
  --field="Force Git Reset:CHK" "FALSE" \
  --field="Build workspace:CHK" "TRUE" \
  --field="SSH into PIs:CHK" "TRUE" \
  --separator="," \
  --button="Run Script":0 --button="Cancel":1)

YAD_EXIT_CODE=$?

if [ "${YAD_EXIT_CODE}" -ne 0 ] || [ -z "${FORM_OUTPUT}" ]; then
  echo "User cancelled or form was empty."
  exit 0
fi

IFS=',' read -r WORKSPACE_FOLDER BRANCH_NAME DDS_CONFIG_FILE QOS_PROFILE QOS_DEPTH PULL_GITHUB FORCE_GIT BUILD_WORKSPACE SSH_PIS <<<"${FORM_OUTPUT}"

echo "Parsed values:"
echo "  WORKSPACE_FOLDER = $WORKSPACE_FOLDER"
echo "  BRANCH_NAME      = $BRANCH_NAME"
echo "  DDS_CONFIG_FILE  = $DDS_CONFIG_FILE"
echo "  QOS_PROFILE      = $QOS_PROFILE"
echo "  QOS_DEPTH        = $QOS_DEPTH"
echo "  PULL_GITHUB      = $PULL_GITHUB"
echo "  FORCE_GIT        = $FORCE_GIT"
echo "  BUILD_WORKSPACE  = $BUILD_WORKSPACE"
echo "  SSH_PIS          = $SSH_PIS"

SRC_PATH="${WORKSPACE_FOLDER}/src"
FORCE_GIT=$([[ "$FORCE_GIT" == "TRUE" ]] && echo "true" || echo "false")

# === Main Execution ===

if [[ "$PULL_GITHUB" == "TRUE" ]]; then
  #sync_git_repo "${SRC_PATH}" "${BRANCH_NAME}" "${FORCE_GIT}"
  for ip in "${!PI_USERS[@]}"; do
    sync_git_repo_ssh "REMOTE $ip" "$ip" "${PI_USERS[$ip]}" "${SRC_PATH}" "${BRANCH_NAME}" "${FORCE_GIT}"
  done
fi

if [[ "${BUILD_WORKSPACE}" == "TRUE" ]]; then
  build_ros_workspace "${WORKSPACE_FOLDER}"
  for ip in "${!PI_USERS[@]}"; do
    build_ros_workspace_ssh "REMOTE $ip" "$ip" "${PI_USERS[$ip]}" "${WORKSPACE_FOLDER}"
  done
fi

if [[ "${SSH_PIS}" == "TRUE" ]]; then
  if tmux has-session -t post_launch 2>/dev/null; then
    tmux kill-session -t post_launch
  fi
  for ip in "${!PI_USERS[@]}"; do
    launch_station_tmux_local "$ip" "${PI_USERS[$ip]}" "${PI_NAMES[$ip]}" "${PI_TYPES[$ip]}" "${WORKSPACE_FOLDER}" "${DDS_CONFIG_FILE}" "${QOS_PROFILE}" "${QOS_DEPTH}"
    sleep(2)
  done
  echo "Local tmux session 'post_launch' created with windows for each Pi."
  echo "Attach using: tmux attach-session -t post_launch"
fi
