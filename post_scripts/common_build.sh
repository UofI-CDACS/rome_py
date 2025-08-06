#!/bin/bash
set -euo pipefail

# Returns build commands as a string for local or remote execution
build_ros_workspace_cmds() {
  local path=$1
  cat <<EOF
set -e
cd "$path"
rm -rf build install log
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon build --symlink-install
EOF
}

# Run build locally
build_ros_workspace() {
  local path=$1
  echo "[LOCAL] Building ROS workspace at $path..."
  eval "$(build_ros_workspace_cmds "$path")"
}

# Run build remotely over ssh_run
build_ros_workspace_ssh() {
  local label=$1
  local ip=$2
  local user=$3
  local path=$4

  if ! declare -f ssh_run >/dev/null; then
    echo "Error: ssh_run function not defined. Please source ssh_common.sh first."
    exit 1
  fi

  echo "[$label] Building ROS workspace remotely at $ip..."
  ssh_run "$ip" "$user" "$(build_ros_workspace_cmds "$path")"
}
