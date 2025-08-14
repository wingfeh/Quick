#!/bin/bash
# Script to start the Mega6 ROS2 control system

echo "--- Stopping any existing Mega6 ROS2 nodes... ---"
pkill -f "m6relay.py" || true
pkill -f "m6sensor.py" || true
sleep 1
echo "Mega6 nodes stopped."
echo ""

# Mega6 System Startup Script
echo "🚀 Starting Mega6 Control System..."
echo "Arduino IP: 192.168.100.106:8888"
echo ""

# Check if Arduino is responsive
echo "Testing Arduino connection..."
timeout 2 bash -c 'echo "STATUS" | nc -u 192.168.100.106 8888' 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Arduino Mega6 is responsive"
else
    echo "⚠️  Arduino Mega6 may not be ready (this is normal if just powered on)"
fi

echo ""
echo "Starting ROS2 nodes..."

# Start the nodes
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega6_control/mega6_control/m6relay.py &
RELAY_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega6_control/mega6_control/m6sensor.py &
SENSOR_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega6_control/mega6_control/mega6_gui.py &
GUI_PID=$!

echo "✅ Mega6 System Started Successfully!"
echo ""
echo "Running processes:"
echo "- m6relay (PID: $RELAY_PID)"
echo "- m6sensor (PID: $SENSOR_PID)"  
echo "- mega6_gui (PID: $GUI_PID)"
echo ""
echo "To stop all: kill $RELAY_PID $SENSOR_PID $GUI_PID"
echo "Or use: pkill -f 'mega6_control|m6'"

# Wait for user input to keep script alive
echo ""
echo "Press Ctrl+C to stop all nodes..."
wait
