#!/bin/bash
# Script to start all eight ROS2 control systems: Mega1, Mega2, Mega3, Mega4, Mega5, Mega6, Mega7, and Mega8

echo "--- Stopping any existing ROS2 nodes... ---"
pkill -f "ros2 launch" || true
pkill -f "m1relay.py" || true
pkill -f "m1sensor.py" || true
pkill -f "m2relay.py" || true
pkill -f "m2sensor.py" || true
pkill -f "m3relay.py" || true
pkill -f "m3sensor.py" || true
pkill -f "m4relay.py" || true
pkill -f "m4sensor.py" || true
pkill -f "m5relay.py" || true
pkill -f "m5sensor.py" || true
pkill -f "m6relay.py" || true
pkill -f "m6sensor.py" || true
pkill -f "m7relay.py" || true
pkill -f "m7sensor.py" || true
pkill -f "m8relay.py" || true
pkill -f "m8sensor.py" || true
pkill -f "mega1_gui.py" || true
pkill -f "mega2_gui.py" || true
pkill -f "mega3_gui.py" || true
pkill -f "mega4_gui.py" || true
pkill -f "mega5_gui.py" || true
pkill -f "mega6_gui.py" || true
pkill -f "mega7_gui.py" || true
pkill -f "mega8_gui.py" || true


sleep 1
echo "All nodes stopped."
echo ""

# --- Environment Setup ---
# (Any environment setup commands would go here)

# Start All Control Systems
echo "Starting All Control Systems..."

echo "Starting Mega1 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/m1relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/m1sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega1_control/mega1_control/mega1_gui.py &

echo "Starting Mega2 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/m2relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/m2sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega2_control/mega2_control/mega2_gui.py &

echo "Starting Mega3 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega3_control/mega3_control/m3relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega3_control/mega3_control/m3sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega3_control/mega3_control/mega3_gui.py &

echo "Starting Mega4 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega4_control/mega4_control/m4relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega4_control/mega4_control/m4sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega4_control/mega4_control/mega4_gui.py &

echo "Starting Mega5 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega5_control/mega5_control/m5relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega5_control/mega5_control/m5sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega5_control/mega5_control/mega5_gui.py &

echo "Starting Mega6 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega6_control/mega6_control/m6relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega6_control/mega6_control/m6sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega6_control/mega6_control/mega6_gui.py &

echo "Starting Mega7 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega7_control/mega7_control/m7relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega7_control/mega7_control/m7sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega7_control/mega7_control/mega7_gui.py &

echo "Starting Mega8 Control System..."
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega8_control/mega8_control/m8relay.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega8_control/mega8_control/m8sensor.py &
python3 /home/wingfeh/Shared/Quick7/ros2_ws/src/mega8_control/mega8_control/mega8_gui.py &

echo ""
echo "🚀 All eight systems started successfully!"
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
echo "Mega3 System:"
echo "- Relay node: m3relay"
echo "- Sensor node: m3sensor"
echo "- GUI: mega3_gui"
echo "- Arduino IP: 192.168.100.103:8888"
echo ""
echo "Mega4 System:"
echo "- Relay node: m4relay"
echo "- Sensor node: m4sensor"
echo "- GUI: mega4_gui"
echo "- Arduino IP: 192.168.100.104:8888"
echo ""
echo "Mega5 System:"
echo "- Relay node: m5relay"
echo "- Sensor node: m5sensor"
echo "- GUI: mega5_gui"
echo "- Arduino IP: 192.168.100.105:8888"
echo ""
echo "Mega6 System:"
echo "- Relay node: m6relay"
echo "- Sensor node: m6sensor"
echo "- GUI: mega6_gui"
echo "- Arduino IP: 192.168.100.106:8888"
echo ""
echo "Mega7 System:"
echo "- Relay node: m7relay"
echo "- Sensor node: m7sensor"
echo "- GUI: mega7_gui"
echo "- Arduino IP: 192.168.100.107:8888"
echo ""
echo "Mega8 System:"
echo "- Relay node: m8relay"
echo "- Sensor node: m8sensor"
echo "- GUI: mega8_gui"
echo "- Arduino IP: 192.168.100.108:8888"
echo ""
echo "Use 'pkill -f python3' to stop all nodes"
