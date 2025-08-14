#!/bin/bash
# Script to start the Mega5 ROS2 control system

echo "--- Stopping any existing Mega5 ROS2 nodes... ---"
pkill -f "m5relay.py" || true
pkill -f "m5sensor.py" || true
sleep 1
echo "Mega5 nodes stopped."
echo ""

# Mega5 System Startup Script
echo "🚀 Starting Mega5 Control System..."
echo "Arduino IP: 192.168.100.105:8888"
echo ""

# Check if Arduino is responsive
echo "Testing Arduino connection..."
timeout 2 bash -c 'echo "STATUS" | nc -u 192.168.100.105 8888' 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Arduino Mega5 is responsive"
else
    echo "⚠️  Arduino Mega5 may not be ready (this is normal if just powered on)"
fi

echo ""
echo "Starting ROS2 nodes..."

# Start the nodes
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega5_control/mega5_control/m5relay.py &
RELAY_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega5_control/mega5_control/m5sensor.py &
SENSOR_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega5_control/mega5_control/mega5_gui.py &
GUI_PID=$!

echo "✅ Mega5 System Started Successfully!"
echo ""
echo "Running processes:"
echo "- m5relay (PID: $RELAY_PID)"
echo "- m5sensor (PID: $SENSOR_PID)"  
echo "- mega5_gui (PID: $GUI_PID)"
echo ""
echo "To stop all: kill $RELAY_PID $SENSOR_PID $GUI_PID"
echo "Or use: pkill -f 'mega5_control|m5'"

# Wait for user input to keep script alive
echo ""
echo "Press Ctrl+C to stop all nodes..."
wait
