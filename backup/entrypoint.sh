#!/bin/bash
set -e

# Source ROS 2 and workspace setup files
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash
# udevadm control --reload # For rpicam

# Start pigpio daemon
sudo pigpiod

# Execute the passed command
exec "$@"
