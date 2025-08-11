#!/bin/bash
# Script to start both Mega1 and Mega2 ROS2 control systems

echo "--- Stopping any existing ROS2 nodes... ---"
pkill -f "ros2 launch" || true
pkill -f "m1relay.py" || true
pkill -f "m1sensor.py" || true
pkill -f "m2relay.py" || true
pkill -f "m2sensor.py" || true
sleep 1
echo "All nodes stopped."
echo ""

# --- Environment Setup ---
# (Any environment setup commands would go here)

# Start Both Mega1 and Mega2 Control Systems
echo "Starting Both Mega1 and Mega2 Control Systems..."

echo "Starting Mega1 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/m1relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/m1sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/mega1_gui.py &

echo "Starting Mega2 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/m2relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/m2sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/mega2_gui.py &

echo ""
echo "🚀 Both systems started successfully!"
echo ""
echo "Mega1 System:"
echo "- Relay node: m1relay"
echo "- Sensor node: m1sensor"
echo "- GUI: mega1_gui"
echo "- Arduino IP: 192.168.100.101:8888"
echo ""
echo "Mega2 System:"
echo "- Relay node: m2relay"
echo "- Sensor node: m2sensor"
echo "- GUI: mega2_gui"
echo "- Arduino IP: 192.168.100.102:8888"
echo ""
echo "Use 'pkill -f python3' to stop all nodes"
