#!/bin/bash
# Script to start the Mega7 ROS2 control system

echo "--- Stopping any existing Mega7 ROS2 nodes... ---"
pkill -f "m7relay.py" || true
pkill -f "m7sensor.py" || true
sleep 1
echo "Mega7 nodes stopped."
echo ""

# Mega7 System Startup Script
echo "🚀 Starting Mega7 Control System..."
echo "Arduino IP: 192.168.100.107:8888"
echo ""

# Check if Arduino is responsive
echo "Testing Arduino connection..."
timeout 2 bash -c 'echo "STATUS" | nc -u 192.168.100.107 8888' 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Arduino Mega7 is responsive"
else
    echo "⚠️  Arduino Mega7 may not be ready (this is normal if just powered on)"
fi

echo ""
echo "Starting ROS2 nodes..."

# Start the nodes
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega7_control/mega7_control/m7relay.py &
RELAY_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega7_control/mega7_control/m7sensor.py &
SENSOR_PID=$!

python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega7_control/mega7_control/mega7_gui.py &
GUI_PID=$!

echo "✅ Mega7 System Started Successfully!"
echo ""
echo "Running processes:"
echo "- m7relay (PID: $RELAY_PID)"
echo "- m7sensor (PID: $SENSOR_PID)"  
echo "- mega7_gui (PID: $GUI_PID)"
echo ""
echo "To stop all: kill $RELAY_PID $SENSOR_PID $GUI_PID"
echo "Or use: pkill -f 'mega7_control|m7'"

# Wait for user input to keep script alive
echo ""
echo "Press Ctrl+C to stop all nodes..."
wait
