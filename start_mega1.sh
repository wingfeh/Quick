#!/bin/bash
# Script to start the Mega1 ROS2 control system

echo "--- Stopping any existing Mega1 ROS2 nodes... ---"
pkill -f "m1relay.py" || true
pkill -f "m1sensor.py" || true
sleep 1
echo "Mega1 nodes stopped."
echo ""

# Mega1 System Startup Script
echo "🚀 Starting Mega1 Control System..."
echo "Arduino IP: 192.168.100.101:8888"
echo ""

# Check if Arduino is responsive
echo "Testing Arduino connection..."
timeout 2 bash -c 'echo "STATUS" | nc -u 192.168.100.101 8888' 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Arduino Mega1 is responsive"
else
    echo "⚠️  Arduino Mega1 may not be ready (this is normal if just powered on)"
fi

echo ""
echo "Starting ROS2 nodes..."

# Start the nodes
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/m1relay.py &
RELAY_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/m1sensor.py &
SENSOR_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/mega1_gui.py &
GUI_PID=$!

echo "✅ Mega1 System Started Successfully!"
echo ""
echo "Running processes:"
echo "- m1relay (PID: $RELAY_PID)"
echo "- m1sensor (PID: $SENSOR_PID)"  
echo "- mega1_gui (PID: $GUI_PID)"
echo ""
echo "To stop all: kill $RELAY_PID $SENSOR_PID $GUI_PID"
echo "Or use: pkill -f 'mega1_control|m1'"

# Wait for user input to keep script alive
echo ""
echo "Press Ctrl+C to stop all nodes..."
wait
