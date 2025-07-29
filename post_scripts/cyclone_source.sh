#!/bin/bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <workspace_dir>"
  exit 1
fi

ws_dir="$1"
dds_file="${ws_dir}/src/post/post_scripts/cyclonedds.xml"
ros_setup="/opt/ros/jazzy/setup.bash"

if [ ! -d "$ws_dir" ]; then
  echo "Error: Workspace directory not found: $ws_dir"
  exit 1
fi

if [ ! -f "$dds_file" ]; then
  echo "Error: CycloneDDS config file not found: $dds_file"
  exit 1
fi

if [ ! -f "$ros_setup" ]; then
  echo "Error: ROS setup.bash not found: $ros_setup"
  exit 1
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$dds_file"
export ROS_DOMAIN_ID=42
source "$ros_setup"

