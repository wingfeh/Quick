#!/usr/bin/env python3
"""
High-Performance Mega7 Sensor Monitoring Node
Monitors sensor on Arduino Mega7 via UDP with 50Hz polling
IP: 192.168.100.107:8888
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import socket
import time

class Mega7SensorNode(Node):
    def __init__(self):
        super().__init__('m7sensor_node')
        
        # Arduino Mega7 configuration
        self.arduino_ip = '192.168.100.107'
        self.arduino_port = 8888
        
        # UDP socket with optimized timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)  # 500ms timeout - standardized with other systems
        
        # Publishers (matching standard pattern)
        self.response_publisher = self.create_publisher(String, 'mega7/sensor_response', 10)
        self.state_publisher = self.create_publisher(Bool, 'mega7/sensor_state', 10)
        
        # Subscriber for sensor commands
        self.command_subscription = self.create_subscription(
            String,
            'mega7/sensor_command',
            self.command_callback,
            10
        )
        
        # State tracking
        self.last_response = "Unknown"
        self.last_state = None
        
        # Timer for periodic sensor readings (high frequency)
        self.sensor_timer = self.create_timer(0.02, self.read_sensor)  # 50Hz
        
        self.get_logger().info('Mega7 Sensor node started. Polling at 50Hz.')
    
    def send_udp_command(self, command):
        """Send UDP command to Arduino and return response"""
        try:
            self.sock.sendto(command.encode(), (self.arduino_ip, self.arduino_port))
            data, addr = self.sock.recvfrom(1024)
            response = data.decode().strip()
            self.get_logger().debug(f'Sent: {command}, Received: {response}')
            return response
        except socket.timeout:
            self.get_logger().debug(f'Timeout reading sensor')
            return None
        except Exception as e:
            self.get_logger().error(f'Error sending command {command}: {e}')
            return None
    
    def command_callback(self, msg):
        """Handle incoming sensor commands"""
        command = msg.data.upper()
        self.get_logger().info(f'Received sensor command: {command}')
        
        if command == 'READ':
            self.read_sensor()
        else:
            self.get_logger().warning(f'Unknown sensor command: {command}')
    
    def read_sensor(self):
        """Read sensor value from Arduino"""
        response = self.send_udp_command('SENSOR')
        if response:
            self.publish_response(response)
            # Parse state from sensor response
            if 'HIGH' in response:
                self.publish_state(True)
            elif 'LOW' in response:
                self.publish_state(False)
    
    def publish_response(self, response_text):
        """Publish sensor response message"""
        if response_text != self.last_response:
            msg = String()
            msg.data = response_text
            self.response_publisher.publish(msg)
            self.last_response = response_text
            self.get_logger().debug(f'Published sensor response: {response_text}')
    
    def publish_state(self, state):
        """Publish boolean sensor state"""
        if state != self.last_state:
            msg = Bool()
            msg.data = state
            self.state_publisher.publish(msg)
            self.last_state = state
            self.get_logger().debug(f'Published sensor state: {state}')

def main(args=None):
    rclpy.init(args=args)
    sensor_node = Mega7SensorNode()
    
    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
