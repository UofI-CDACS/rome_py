#!/bin/bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/src/post/post_scripts/fastdds_unicast.xml"
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
