#!/bin/bash
set -euo pipefail

# === Configurable Mappings ===
declare -A PI_USERS=(
  [172.23.254.18]="rospi"
  [172.23.254.22]="rospi"
  [172.23.254.23]="rospi"
  [172.23.254.24]="rospi"
)

declare -A PI_NAMES=(
  [172.23.254.24]="rospi_1"
  [172.23.254.22]="rospi_2"
  [172.23.254.23]="rospi_3"
  [172.23.254.18]="rospi_4"
)

# === Git Functions ===
sync_git_repo_block() {
  echo "[$1] Syncing repo..."
  local ip="$2"
  local user="$3"
  local path="$4"
  local branch="$5"

  ssh_run "$ip" "$user" "
    set -e
    cd \"$path\"
    if [ ! -d post ]; then
      git clone https://github.com/UofI-CDACS/rome_py.git post
    fi
    cd post
    git fetch --all
    current_branch=\$(git rev-parse --abbrev-ref HEAD)
    if [ \"\$current_branch\" != \"$branch\" ]; then
      git checkout \"$branch\"
    fi
    git reset --hard origin/$branch
    git pull origin $branch
  "
}

sync_git_repo() {
  local path="$1"
  local branch="$2"

  cd "$path"
  if [ ! -d post ]; then
    git clone https://github.com/UofI-CDACS/rome_py.git post
  fi
  cd post
  git fetch --all
  local current_branch
  current_branch=$(git rev-parse --abbrev-ref HEAD)
  if [ "$current_branch" != "$branch" ]; then
    git checkout "$branch"
  fi
  git reset --hard "origin/$branch"
  git pull origin "$branch"
}

# === Build Functions ===
build_ros_workspace_block() {
  echo "[$1] Building workspace..."
  local ip="$2"
  local user="$3"
  local path="$4"

  ssh_run "$ip" "$user" "
    set -e
    cd \"$path\"
    rm -rf build install log
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon build --symlink-install
  "
}

build_ros_workspace() {
  local path="$1"

  cd "$path"
  rm -rf build install log
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  colcon build --symlink-install
}

# === Utility ===
ssh_run() {
  local ip="$1"
  local user="$2"
  local script="$3"

  ssh -A -o StrictHostKeyChecking=no "${user}@${ip}" bash -s <<EOF
$script
EOF
}

# === Launch using local tmux session ===
launch_station_tmux_local() {
  local ip="$1"
  local user="$2"
  local node_name="$3"
  local ws_path="$4"
  local dds_config="$5"
  local qos_profile="$6"
  local qos_depth="$7"

  local session_name="post_launch"
  local window_name="${ip//./_}"

  # Create tmux session if it does not exist
  if ! tmux has-session -t "$session_name" 2>/dev/null; then
    tmux new-session -d -s "$session_name" -n "$window_name"
  else
    # If window exists, kill and recreate for fresh start
    if tmux list-windows -t "$session_name" | grep -q "^$window_name"; then
      tmux kill-window -t "${session_name}:$window_name"
    fi
    tmux new-window -t "$session_name" -n "$window_name"
  fi

  # Send SSH and launch command to tmux window
  # Build the remote launch command
  remote_cmd="cd \"$ws_path\""
  remote_cmd+=" && source \"$ws_path/src/post/post_scripts/$dds_config\""
  remote_cmd+=" && source \"$ws_path/install/setup.bash\""
  remote_cmd+=" && ros2 run post_core station"
  remote_cmd+=" --name ${node_name}"
  remote_cmd+=" --type default"
  remote_cmd+=" --lossmode ${qos_profile}"
  remote_cmd+=" --depth ${qos_depth}"

  # Send the SSH + launch command into the tmux window
  tmux send-keys -t "${session_name}:${window_name}" \
    "ssh -o StrictHostKeyChecking=no ${user}@${ip} bash -c '$remote_cmd'" C-m
}

# === YAD Form ===
DEFAULT_WORKSPACE="$HOME/Desktop/test_ws"
DEFAULT_BRANCH="post-develop-branch"
DEFAULT_DDS="cyclone_source.sh"
DEFAULT_QOS_PROFILE="lossless"
DEFAULT_QOS_DEPTH="10"

FORM_OUTPUT=$(yad --form --title="Launch Parcel Script" \
  --text="Configure Launch Parameters" \
  --field="Workspace Folder:TXT" "$DEFAULT_WORKSPACE" \
  --field="Branch Name:TXT" "$DEFAULT_BRANCH" \
  --field="DDS Config:CB" "cyclone_source.sh!fast_source.sh" \
  --field="QOS Profile:CB" "lossless!lossy" \
  --field="QOS Depth:NUM" "$DEFAULT_QOS_DEPTH" \
  --field="Pull from GitHub:CHK" "TRUE" \
  --field="Build workspace:CHK" "TRUE" \
  --field="SSH into PIs:CHK" "TRUE" \
  --separator="," \
  --button="Run Script":0 --button="Cancel":1)

YAD_EXIT_CODE=$?

if [ $YAD_EXIT_CODE -ne 0 ] || [ -z "$FORM_OUTPUT" ]; then
  echo "User cancelled or form was empty."
  exit 0
fi

IFS=',' read -r WORKSPACE_FOLDER BRANCH_NAME DDS_CONFIG_FILE QOS_PROFILE QOS_DEPTH PULL_GITHUB BUILD_WORKSPACE SSH_PIS <<<"$FORM_OUTPUT"

SRC_PATH="${WORKSPACE_FOLDER}/src"

# === Main Execution ===

if [ "$PULL_GITHUB" = "TRUE" ]; then
  sync_git_repo "$SRC_PATH" "$BRANCH_NAME"
  for ip in "${!PI_USERS[@]}"; do
    sync_git_repo_block "REMOTE $ip" "$ip" "${PI_USERS[$ip]}" "$SRC_PATH" "$BRANCH_NAME"
  done
fi

if [ "$BUILD_WORKSPACE" = "TRUE" ]; then
  build_ros_workspace "$WORKSPACE_FOLDER"
  for ip in "${!PI_USERS[@]}"; do
    build_ros_workspace_block "REMOTE $ip" "$ip" "${PI_USERS[$ip]}" "$WORKSPACE_FOLDER"
  done
fi

if [ "$SSH_PIS" = "TRUE" ]; then
  for ip in "${!PI_USERS[@]}"; do
    launch_station_tmux_local "$ip" "${PI_USERS[$ip]}" "${PI_NAMES[$ip]}" "$WORKSPACE_FOLDER" "$DDS_CONFIG_FILE" "$QOS_PROFILE" "$QOS_DEPTH"
  done
  echo "Local tmux session 'post_launch' created with windows for each Pi."
  echo "Attach using: tmux attach-session -t post_launch"
fi
