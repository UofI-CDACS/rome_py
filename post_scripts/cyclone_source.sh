#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$(pwd)/src/post/post_scripts/cyclonedds_unicast.xml"
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash