#!/bin/bash
# Script to start the Mega8 ROS2 control system

echo "--- Stopping any existing Mega8 ROS2 nodes... ---"
pkill -f "m8relay.py" || true
pkill -f "m8sensor.py" || true
sleep 1
echo "Mega8 nodes stopped."
echo ""

# Mega8 System Startup Script
echo "🚀 Starting Mega8 Control System..."
echo "Arduino IP: 192.168.100.108:8888"
echo ""

# Check if Arduino is responsive
echo "Testing Arduino connection..."
timeout 2 bash -c 'echo "STATUS" | nc -u 192.168.100.108 8888' 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Arduino Mega8 is responsive"
else
    echo "⚠️  Arduino Mega8 may not be ready (this is normal if just powered on)"
fi

echo ""
echo "Starting ROS2 nodes..."

# Start the nodes
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega8_control/mega8_control/m8relay.py &
RELAY_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega8_control/mega8_control/m8sensor.py &
SENSOR_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega8_control/mega8_control/mega8_gui.py &
GUI_PID=$!

echo "✅ Mega8 System Started Successfully!"
echo ""
echo "Running processes:"
echo "- m8relay (PID: $RELAY_PID)"
echo "- m8sensor (PID: $SENSOR_PID)"  
echo "- mega8_gui (PID: $GUI_PID)"
echo ""
echo "To stop all: kill $RELAY_PID $SENSOR_PID $GUI_PID"
echo "Or use: pkill -f 'mega8_control|m8'"

# Wait for user input to keep script alive
echo ""
echo "Press Ctrl+C to stop all nodes..."
wait
