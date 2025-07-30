#!/bin/bash
set -euo pipefail

# Print usage info and exit
usage() {
  echo "Usage: $0 <workspace_dir> [-u|--unicast]"
  exit 1
}

# Check arguments count
check_args() {
  if [ $# -lt 1 ]; then
    usage
  fi
}

# Parse command-line arguments
parse_args() {
  ws_dir=""
  use_unicast=false

  while (($#)); do
    case $1 in
    -u | --unicast)
      use_unicast=true
      shift
      ;;
    -*)
      echo "Unknown option: $1"
      usage
      ;;
    *)
      if [[ -z $ws_dir ]]; then
        ws_dir=$1
        shift
      else
        echo "Unexpected arg: $1"
        usage
      fi
      ;;
    esac
  done
  echo "$ws_dir" "$use_unicast"
}

# Check if directory exists
check_dir() {
  local dir=$1
  if [[ ! -d $dir ]]; then
    echo "Workspace not found: $dir"
    exit 1
  fi
}

# Check if file exists
check_file() {
  local file=$1
  if [[ ! -f $file ]]; then
    echo "Missing file: $file"
    exit 1
  fi
}

main() {
  check_args "$@"
  # parse_args returns two values: workspace dir and use_unicast flag
  read -r ws_dir use_unicast <<<"$(parse_args "$@")"

  check_dir "$ws_dir"

  local dds_file="$ws_dir/src/post/post_scripts/cyclonedds_$($use_unicast && echo unicast || echo multicast).xml"
  local ros_setup="/opt/ros/jazzy/setup.bash"

  check_file "$dds_file"
  check_file "$ros_setup"

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="$dds_file"
  export ROS_DOMAIN_ID=42

  # Source ROS setup.bash with disabled -u temporarily
  set +u
  source "$ros_setup"
  set -u
}

main "$@"
