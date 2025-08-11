# Quick7 - Dual Arduino Mega Ethernet Control System

A high-performance dual Arduino Mega 2560 system for real-time relay and sensor control over Ethernet with ROS2 integration.

## 🚀 **System Overview**

- **Arduino Mega1** (192.168.100.101) - `/dev/ttyACM0`
- **Arduino Mega2** (192.168.100.102) - `/dev/ttyACM1`
- **ROS2 Jazzy** integration with dedicated control nodes
- **Sub-millisecond** response times via optimized UDP protocol

## 📁 **Project Structure**

```
Quick7/
├── arduino-mega-dual/          # Unified Arduino firmware project
│   ├── src/
│   │   ├── mega1.cpp          # Mega1 firmware (IP .101)
│   │   └── mega2.cpp          # Mega2 firmware (IP .102)
│   ├── platformio.ini         # Build configuration
│   └── README.md              # Arduino project docs
├── ros2_ws/                   # ROS2 workspace
│   └── src/
│       ├── mega1_control/     # Mega1 ROS2 package
│       └── mega2_control/     # Mega2 ROS2 package
├── start_mega1.sh            # Mega1 system startup
├── start_mega2.sh            # Mega2 system startup
└── start_both_systems.sh     # Start both systems
```

## ⚡ **Quick Start**

### Arduino Firmware Upload
```bash
cd arduino-mega-dual

# Upload to Mega1
pio run -e mega1 --target upload --upload-port /dev/ttyACM0

# Upload to Mega2  
pio run -e mega2 --target upload --upload-port /dev/ttyACM1
```

### ROS2 System Launch
```bash
# Start Mega1 system
./start_mega1.sh

# Start Mega2 system
./start_mega2.sh

# Start both systems
./start_both_systems.sh
```

### Direct UDP Testing
```bash
# Test Mega1
echo "STATUS" | nc -u 192.168.100.101 8888
echo "RELAY_ON" | nc -u 192.168.100.101 8888
echo "SENSOR" | nc -u 192.168.100.101 8888

# Test Mega2
echo "STATUS" | nc -u 192.168.100.102 8888
echo "RELAY_ON" | nc -u 192.168.100.102 8888
echo "SENSOR" | nc -u 192.168.100.102 8888
```

## 🔧 **Hardware Configuration**

| Component | Mega1 | Mega2 |
|-----------|-------|-------|
| **IP Address** | 192.168.100.101 | 192.168.100.102 |
| **MAC Address** | DE:AD:BE:EF:FE:01 | DE:AD:BE:EF:FE:02 |
| **Relay Pin** | Pin 4 | Pin 4 |
| **Sensor Pin** | Pin 8 | Pin 8 |
| **USB Port** | /dev/ttyACM0 | /dev/ttyACM1 |

## 📡 **UDP Commands**

- `STATUS` - Get relay state (ON/OFF)
- `RELAY_ON` - Turn relay on
- `RELAY_OFF` - Turn relay off  
- `SENSOR` - Read sensor state (HIGH/LOW)

## 🏗️ **Architecture**

- **Ultra-fast UDP protocol** - Optimized for minimal latency
- **ROS2 integration** - Professional robotics middleware
- **Dual network isolation** - Separate /mega1/ and /mega2/ namespaces
- **Serial debugging** - Real-time command monitoring
- **Automated startup** - Background process management

## ✅ **System Status**

Both Arduino systems are fully operational with reliable upload capabilities and network communication.
