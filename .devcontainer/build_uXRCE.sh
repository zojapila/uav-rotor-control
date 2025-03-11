#!/bin/bash
set -e

# Build Micro-XRCE-DDS-Agent
echo "*********************************"
echo "* Building Micro-XRCE-DDS-Agent *"

cd /home/developer/ros2_ws/src/vcs_repos/Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
make install
ldconfig /usr/local/lib/

echo "* Building Micro-XRCE-DDS-Agent COMPLETED *"
echo "*******************************************"