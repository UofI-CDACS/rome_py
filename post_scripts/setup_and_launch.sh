#!/bin/bash
set -euo pipefail

# --- MACHINE INFO: IP to user and node name mappings ---
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

# --- FUNCTION: Sync local or remote post repo to match remote branch ---
sync_git_repo() {
  local path="$1"
  local branch="$2"

  cd "$path"

  # Clone if missing
  if [ ! -d post ]; then
    git clone https://github.com/UofI-CDACS/rome_py.git post
  fi

  cd post

  # Ensure we're on correct branch and synced
  git fetch --all
  local current_branch
  current_branch=$(git rev-parse --abbrev-ref HEAD)

  if [ "$current_branch" != "$branch" ]; then
    git checkout "$branch"
  fi

  git reset --hard "origin/$branch"
  git pull origin "$branch"
}

# --- FUNCTION: Clean and build ROS workspace ---
build_ros_workspace() {
  local path="$1"

  cd "$path"
  rm -rf build install log
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
}

# --- FUNCTION: Run given script string remotely via SSH with safe flags ---
ssh_run() {
  local ip="$1"
  local user="$2"
  local script="$3"

  ssh -A -tt -o StrictHostKeyChecking=no "${user}@${ip}" bash -c "$script"
}

# --- FUNCTION: Open terminal tab and launch station node remotely ---
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
        ssh -tt -o StrictHostKeyChecking=no ${user}@${ip} bash -c '
            cd \"$ws_path\"
            chmod -R +rwx .
            source \"./src/post/post_scripts/${dds_config}\"
            source \"$ws_path/install/setup.bash\"
            ros2 run post_core station \
                --name ${node_name} \
                --type default \
                --lossmode ${qos_profile} \
                --depth ${qos_depth}
            bash
        ';
        bash
    "
}

# ========================
# === YAD FORM INPUT ====
# ========================

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

# If cancel or close
if [ $YAD_EXIT_CODE -ne 0 ] || [ -z "$FORM_OUTPUT" ]; then
  echo "User cancelled or form was empty."
  exit 0
fi

# Parse form output
IFS=',' read -r \
  WORKSPACE_FOLDER \
  BRANCH_NAME \
  DDS_CONFIG_FILE \
  QOS_PROFILE \
  QOS_DEPTH \
  PULL_GITHUB \
  BUILD_WORKSPACE \
  SSH_PIS \
  <<<"$FORM_OUTPUT"

SRC_PATH="${WORKSPACE_FOLDER}/src"

# =====================
# === MAIN EXECUTION ==
# =====================

# -- Step 1: Git Pull --
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

# -- Step 2: Build --
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
            source /opt/ros/jazzy/setup.bash
            colcon build --symlink-install
        "
  done
fi

# -- Step 3: Launch nodes --
if [ "$SSH_PIS" = "TRUE" ]; then
  for ip in "${!PI_USERS[@]}"; do
    user="${PI_USERS[$ip]}"
    node_name="${PI_NAMES[$ip]}"
    echo "[REMOTE $ip] Launching station node..."
    launch_station "$ip" "$user" "$node_name" "$WORKSPACE_FOLDER" "$DDS_CONFIG_FILE" "$QOS_PROFILE" "$QOS_DEPTH"
  done
fi
