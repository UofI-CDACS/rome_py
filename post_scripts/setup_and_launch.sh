#!/bin/bash
set -euo pipefail

declare -A PI_USERS=(
  [172.23.254.18]="rospi"
  [172.23.254.22]="rospi"
  [172.23.254.23]="rospi"
  [172.23.254.24]="rospi"
)

declare -A PI_NAMES=(
  [172.23.254.18]="rospi_1"
  [172.23.254.22]="rospi_2"
  [172.23.254.23]="rospi_3"
  [172.23.254.24]="rospi_4"
)

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

build_ros_workspace() {
  local path="$1"

  cd "$path"
  rm -rf build install log

  # Temporarily disable 'set -u' to avoid unbound variable error in ROS setup.bash
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u

  colcon build --symlink-install
}

# Use heredoc to send script safely to ssh without -c quoting issues
ssh_run() {
  local ip="$1"
  local user="$2"
  local script="$3"

  ssh -A -tt -o StrictHostKeyChecking=no "${user}@${ip}" bash -s <<EOF
$script
EOF
}

launch_station() {
  local ip="$1"
  local user="$2"
  local node_name="$3"
  local ws_path="$4"
  local dds_config="$5"
  local qos_profile="$6"
  local qos_depth="$7"

  gnome-terminal --tab -- bash -c "
        echo 'Connecting to ${user}@${ip}...';
        ssh -tt -o StrictHostKeyChecking=no ${user}@${ip} bash -s <<EOF
cd \"$ws_path\"
chmod -R +rwx .
source \"./src/post/post_scripts/${dds_config}\"
source \"$ws_path/install/setup.bash\"
ros2 run post_core station --name ${node_name} --type default --lossmode ${qos_profile} --depth ${qos_depth}
bash
EOF
        ;
        bash
    "
}

# === YAD form ===

DEFAULT_WORKSPACE="$HOME/Desktop/test_ws"
DEFAULT_BRANCH="post-develop-branch"
DEFAULT_DDS="cyclone_source.sh"
DEFAULT_QOS_PROFILE="lossless"
DEFAULT_QOS_DEPTH="10"

FORM_OUTPUT=$(
  yad --form --title="Launch Parcel Script" \
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
    --button="Run Script":0 --button="Cancel":1
)

YAD_EXIT_CODE=$?

if [ $YAD_EXIT_CODE -ne 0 ] || [ -z "$FORM_OUTPUT" ]; then
  echo "User cancelled or form was empty."
  exit 0
fi

IFS=',' read -r WORKSPACE_FOLDER BRANCH_NAME DDS_CONFIG_FILE QOS_PROFILE QOS_DEPTH PULL_GITHUB BUILD_WORKSPACE SSH_PIS <<<"$FORM_OUTPUT"

SRC_PATH="${WORKSPACE_FOLDER}/src"

# --- Main logic ---

if [ "$PULL_GITHUB" = "TRUE" ]; then
  echo "[LOCAL] Syncing repo..."
  sync_git_repo "$SRC_PATH" "$BRANCH_NAME"

  for ip in "${!PI_USERS[@]}"; do
    user="${PI_USERS[$ip]}"
    echo "[REMOTE $ip] Syncing repo..."
    ssh_run "$ip" "$user" "
set -e
cd \"$SRC_PATH\"
if [ ! -d post ]; then
    git clone https://github.com/UofI-CDACS/rome_py.git post
fi
cd post
git fetch --all
current_branch=\$(git rev-parse --abbrev-ref HEAD)
if [ \"\$current_branch\" != \"$BRANCH_NAME\" ]; then
    git checkout \"$BRANCH_NAME\"
fi
git reset --hard origin/$BRANCH_NAME
git pull origin $BRANCH_NAME
"
  done
fi

if [ "$BUILD_WORKSPACE" = "TRUE" ]; then
  echo "[LOCAL] Building workspace..."
  build_ros_workspace "$WORKSPACE_FOLDER"

  for ip in "${!PI_USERS[@]}"; do
    user="${PI_USERS[$ip]}"
    echo "[REMOTE $ip] Building workspace..."
    ssh_run "$ip" "$user" "
set -e
cd \"$WORKSPACE_FOLDER\"
rm -rf build install log

# Disable 'set -u' before sourcing ROS setup on remote to avoid unbound var error
set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon build --symlink-install
"
  done
fi

if [ "$SSH_PIS" = "TRUE" ]; then
  for ip in "${!PI_USERS[@]}"; do
    user="${PI_USERS[$ip]}"
    node_name="${PI_NAMES[$ip]}"
    echo "[REMOTE $ip] Launching station node..."
    launch_station "$ip" "$user" "$node_name" "$WORKSPACE_FOLDER" "$DDS_CONFIG_FILE" "$QOS_PROFILE" "$QOS_DEPTH"
  done
fi
