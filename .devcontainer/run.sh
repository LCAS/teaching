#!/bin/bash

set -e

WORKSPACE="`pwd`"

git config --global core.autocrlf false

sudo chown -R lcas /home/lcas/ws
sudo chown -R lcas /workspaces

source /opt/ros/lcas/install/setup.bash
sudo apt update
rosdep --rosdistro=humble update 

cd /home/lcas/ws
colcon build --symlink-install

if ! grep -q "source /opt/ros/lcas/install/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/lcas/install/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source /home/lcas/ws/install/setup.bash" ~/.bashrc; then
    echo "source /home/lcas/ws/install/setup.bash" >> ~/.bashrc
fi
