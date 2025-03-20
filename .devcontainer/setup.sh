#!/bin/bash
set -e

if [ $# -gt 0 ]; then
    if [ $1 -gt 0 ]; then
        echo "export ROS_DOMAIN_ID=$1" >> ~/.bashrc
        export ROS_DOMAIN_ID=$1
    fi
else
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
    export ROS_DOMAIN_ID=0
fi
echo "Using ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

cd /home/developer/ros2_ws

vcs import --recursive < /home/developer/ros2_ws/src/px4.repos src

rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
