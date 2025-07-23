#!/bin/bash
set -euo pipefail

# Defaults
REMOTE_USER="rospi"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP="$HOME/ws/install/setup.bash"
HOSTS=("172.23.254.20" "172.23.254.24" "172.23.254.22" "172.23.254.23" "172.23.254.18")
NODES=("rospi_0" "rospi_1" "rospi_2" "rospi_3" "rospi_4")

usage() {
  cat <<EOF
Usage: $0 [-u remote_user] [-r ros_setup] [-w workspace_setup] [-h hosts_csv] [-n nodes_csv]

  -u  Remote SSH user (default: $REMOTE_USER)
  -r  Path to ROS setup script on remote (default: $ROS_SETUP)
  -w  Path to workspace setup script on remote (default: $WORKSPACE_SETUP)
  -h  Comma-separated list of hosts (default: $HOSTS)
  -n  Comma-separated list of node names (default: $NODES)

Example:
  $0 -u rospi -r /opt/ros/jazzy/setup.bash -w /home/rospi/ws/install/setup.bash -h rospi-1,rospi-2 -n node1,node2
EOF
  exit 1
}

# Parse options
while getopts ":u:r:w:h:n:" opt; do
  case $opt in
  u) REMOTE_USER="$OPTARG" ;;
  r) ROS_SETUP="$OPTARG" ;;
  w) WORKSPACE_SETUP="$OPTARG" ;;
  h) IFS=',' read -r -a HOSTS_ARRAY <<<"$OPTARG" ;;
  n) IFS=',' read -r -a NODES_ARRAY <<<"$OPTARG" ;;
  *) usage ;;
  esac
done

# Use defaults if arrays not set by flags
if [ -z "${HOSTS_ARRAY+x}" ]; then
  IFS=',' read -r -a HOSTS_ARRAY <<<"$HOSTS"
fi

if [ -z "${NODES_ARRAY+x}" ]; then
  IFS=',' read -r -a NODES_ARRAY <<<"$NODES"
fi

# Validate equal length
if [ "${#HOSTS_ARRAY[@]}" -ne "${#NODES_ARRAY[@]}" ]; then
  echo "Error: Number of hosts and nodes must be equal."
  exit 1
fi

# Debug print arrays (optional)
echo "Remote user: $REMOTE_USER"
echo "ROS setup script: $ROS_SETUP"
echo "Workspace setup script: $WORKSPACE_SETUP"
echo "Hosts: ${HOSTS_ARRAY[*]}"
echo "Nodes: ${NODES_ARRAY[*]}"

# Launch loop
for i in "${!HOSTS_ARRAY[@]}"; do
  HOST="${HOSTS_ARRAY[$i]}"
  NODE="${NODES_ARRAY[$i]}"

  echo "Launching $NODE on $HOST..."

  gnome-terminal --title="${NODE}@${HOST}" -- bash -c "
    ssh ${REMOTE_USER}@${HOST} '
      set -e
      source \"$ROS_SETUP\"
      source \"$WORKSPACE_SETUP\"
      ros2 run post_station station --name \"$NODE\" --type default
    '
    exec bash
  "
done
