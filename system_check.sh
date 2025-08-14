#!/bin/bash
# system_check.sh - Quick system status check for all mega systems

echo "=== Quick7 System Status Check ==="
echo

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced. Sourcing workspace..."
    cd /home/wingfeh/Shared/Quick7/ros2_ws
    source install/setup.bash
else
    echo "✅ ROS2 environment: $ROS_DISTRO"
fi

echo
echo "=== Network Connectivity Test ==="
for i in {1..6}; do
    ip="192.168.100.10$i"
    if ping -c 1 -W 1 $ip > /dev/null 2>&1; then
        echo "✅ Mega$i ($ip) - Network OK"
    else
        echo "❌ Mega$i ($ip) - Network FAIL"
    fi
done

echo
echo "=== ROS2 Topic Check ==="
echo "Active topics:"
timeout 3 ros2 topic list | grep -E "mega[1-6]" | head -30

echo
echo "=== Process Check ==="
echo "Running ROS2 nodes:"
ps aux | grep -E "(m[1-6](relay|sensor)|mega[1-6])" | grep -v grep | wc -l | xargs echo "Active nodes:"

echo
echo "=== Quick Performance Test ==="
echo "To run performance tests:"
echo "  Relay speed:        python3 /home/wingfeh/Shared/Quick7/performance_test.py relay_speed"
echo "  Concurrent relays:  python3 /home/wingfeh/Shared/Quick7/performance_test.py concurrent_relays"
echo "  Sensor monitoring:  python3 /home/wingfeh/Shared/Quick7/performance_test.py sensor_monitoring"
echo "  Full stress test:   python3 /home/wingfeh/Shared/Quick7/performance_test.py full_stress"
echo
