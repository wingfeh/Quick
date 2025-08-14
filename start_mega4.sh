#!/bin/bash
# start_mega4.sh - Launch Mega4 control system

echo "Starting Mega4 Control System..."

# Kill any existing nodes to avoid conflicts
pkill -f "m4relay"
pkill -f "m4sensor" 
pkill -f "mega4_gui"

# Small delay to ensure cleanup
sleep 1

# Source ROS2 workspace
cd /home/wingfeh/Shared/Quick7/ros2_ws
source install/setup.bash

# Start nodes in background
echo "Starting Mega4 relay node..."
ros2 run mega4_control m4relay &

echo "Starting Mega4 sensor node..."
ros2 run mega4_control m4sensor &

sleep 2

echo "Starting Mega4 GUI..."
ros2 run mega4_control mega4_gui &

echo "Mega4 system started! Check GUI window."
echo "Press Ctrl+C to stop all nodes."

# Wait for user interrupt
wait
