#!/bin/bash

set -e
source /opt/ros/humble/setup.bash
sudo apt update
rosdep --rosdistro=humble update 

cd /home/lcas/ws
colcon build 
echo "source /home/lcas/ws/install/setup.bash" >> ~/.bashrc

