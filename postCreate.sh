#!/bin/sh
# Set permissions of workspace in order for rosdep to functional properly
echo "Setting permissions for workspace"
chown -R ros2 /home/ws/