#!/bin/bash
set -e

# ROS 2 Setup
source /opt/ros/jazzy/setup.bash

# Workspace Setup (falls gebaut)
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

# Colcon Autocomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

exec "$@"
