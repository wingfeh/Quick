#!/bin/bash
# Script to start the Mega3 ROS2 control system

echo "--- Stopping any existing Mega3 ROS2 nodes... ---"
pkill -f "m3relay.py" || true
pkill -f "m3sensor.py" || true
pkill -f "mega3_gui.py" || true
sleep 1
echo "Mega3 nodes stopped."
echo ""

# --- Environment Setup ---
source /opt/ros/jazzy/setup.bash
source /home/wingfeh/Shared/Quick7/ros2_ws/install/setup.bash

# Start Mega3 Control System
echo "Starting Mega3 Control System..."
ros2 launch mega3_control mega3_system.launch.py &

echo ""
echo "🚀 Mega3 system started successfully!"
echo ""
echo "Mega3 System:"
echo "- Relay node: m3relay"
echo "- Sensor node: m3sensor"
echo "- GUI: mega3_gui"
echo "- Arduino IP: 192.168.100.103:8888"
echo ""
echo "Use 'pkill -f m3' to stop nodes"
