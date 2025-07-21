#!/bin/bash
export RMW_IMPLEMENTATION=rmw_fastdds_cpp
export CYCLONEDDS_URI=file://$1/fastdds.xml
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
