#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/common.sh"
source "$SCRIPT_DIR/common_dds.sh"

main() {
  check_args "$@"
  read -r ws_dir use_unicast <<<"$(parse_args "$@")"

  check_dir "$ws_dir"

  local config_file="${ws_dir}/src/post/post_scripts/zenohdds_$([ "$use_unicast" = true ] && echo unicast || echo multicast).json"
  local ros_setup="/opt/ros/jazzy/setup.bash"

  check_file "$config_file"
  check_file "$ros_setup"

  export RMW_IMPLEMENTATION=rmw_zenohdds_cpp
  export ZENOHDDS_URI="$config_file"
  export ROS_DOMAIN_ID=42

  set +u
  source "$ros_setup"
  set -u
}

main "$@"
