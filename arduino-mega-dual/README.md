# Arduino Mega Dual System

This folder contains the firmware for both Arduino Mega controllers in a unified project structure.

## Files Structure
```
arduino-mega-dual/
├── platformio.ini          # Unified build configuration
├── src/
│   ├── mega1.cpp           # Mega1 firmware (IP: 192.168.100.101)
│   └── mega2.cpp           # Mega2 firmware (IP: 192.168.100.102)
└── README.md
```

## Configuration Summary

| Arduino | File | IP Address | MAC Address | Port Assignment |
|---------|------|------------|-------------|-----------------|
| **Mega1** | `mega1.cpp` | 192.168.100.101 | 0xFE, 0x01 | Usually /dev/ttyACM1 |
| **Mega2** | `mega2.cpp` | 192.168.100.102 | 0xFE, 0x02 | Usually /dev/ttyACM0 |

## Build and Upload Commands

### For Mega1 (192.168.100.101):
```bash
cd /home/wingfeh/Shared/Quick7/arduino-mega-dual

# Build Mega1
pio run -e mega1

# Upload to Mega1 (specify correct port)
pio run -e mega1 --target upload --upload-port /dev/ttyACM1

# Monitor Mega1 serial
pio device monitor --port /dev/ttyACM1 --baud 115200
```

### For Mega2 (192.168.100.102):
```bash
cd /home/wingfeh/Shared/Quick7/arduino-mega-dual

# Build Mega2
pio run -e mega2

# Upload to Mega2 (specify correct port)
pio run -e mega2 --target upload --upload-port /dev/ttyACM0

# Monitor Mega2 serial
pio device monitor --port /dev/ttyACM0 --baud 115200
```

## Quick Upload Scripts

### Upload to Both Arduinos:
```bash
# Upload Mega1
pio run -e mega1 --target upload --upload-port /dev/ttyACM1

# Upload Mega2  
pio run -e mega2 --target upload --upload-port /dev/ttyACM0
```

## Testing Connectivity

```bash
# Test Mega1
echo "STATUS" | nc -u 192.168.100.101 8888

# Test Mega2
echo "STATUS" | nc -u 192.168.100.102 8888
```

## Features
- **Unified project structure** - Single folder for both Arduinos
- **Clear file naming** - mega1.cpp and mega2.cpp
- **Environment-specific builds** - Use `-e mega1` or `-e mega2`
- **Optimized firmware** - Sub-millisecond response times
- **Serial debugging** - Monitor real-time command processing
