#!/bin/bash

# source additional packages
source /opt/ros/lcas/install/setup.bash

# ensure students always have the latest index of everything
sudo apt update
rosdep --rosdistro=humble update

# ensure the additional packages are always sources:
if ! grep -q "source /opt/ros/lcas/install/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/lcas/install/setup.bash" >> ~/.bashrc
fi
