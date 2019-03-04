#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "/home/odroid/eurobot_2018/high_level/devel/setup.sh"
source  /home/odroid/eurobot_2018/high_level/devel/setup.bash
export ROS_MASTER_URI=http://10.0.0.26:11311
export ROS_IP=10.0.0.34
export ROS_HOSTNAME=10.0.0.34
exec "$@"
