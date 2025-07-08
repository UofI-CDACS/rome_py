#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
