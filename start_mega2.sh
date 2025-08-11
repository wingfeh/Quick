#!/bin/bash

# Script to start the Mega2 ROS2 control system

echo "--- Stopping any existing Mega2 ROS2 nodes... ---"
pkill -f "m2relay.py" || true
pkill -f "m2sensor.py" || true
sleep 1
echo "Mega2 nodes stopped."
echo ""

# Mega2 System Startup Script
echo "🚀 Starting Mega2 Control System..."
echo "Arduino IP: 192.168.100.102:8888"
echo ""

# Check if Arduino is responsive
echo "Testing Arduino connection..."
timeout 2 bash -c 'echo "STATUS" | nc -u 192.168.100.102 8888' 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Arduino Mega2 is responsive"
else
    echo "⚠️  Arduino Mega2 may not be ready (this is normal if just powered on)"
fi

echo ""
echo "Starting ROS2 nodes..."

# Start the nodes
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/m2relay.py &
RELAY_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/m2sensor.py &
SENSOR_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/mega2_gui.py &
GUI_PID=$!

echo "✅ Mega2 System Started Successfully!"
echo ""
echo "Running processes:"
echo "- m2relay (PID: $RELAY_PID)"
echo "- m2sensor (PID: $SENSOR_PID)"  
echo "- mega2_gui (PID: $GUI_PID)"
echo ""
echo "To stop all: kill $RELAY_PID $SENSOR_PID $GUI_PID"
echo "Or use: pkill -f 'mega2_control|m2'"

# Wait for user input to keep script alive
echo ""
echo "Press Ctrl+C to stop all nodes..."
wait
