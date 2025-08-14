#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
import time

class Mega5SensorNode(Node):
    def __init__(self):
        super().__init__('m5sensor_node')
        
        # Publishers (matching GUI expectations)
        self.response_publisher = self.create_publisher(String, 'mega5/sensor_response', 10)
        self.state_publisher = self.create_publisher(Bool, 'mega5/sensor_state', 10)
        
        # Subscriber for sensor commands
        self.command_subscription = self.create_subscription(
            String,
            'mega5/sensor_command',
            self.command_callback,
            10
        )
        
        # Arduino connection settings
        self.arduino_ip = '192.168.100.105'
        self.arduino_port = 8888
        
        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        
        # State tracking
        self.last_response = "Unknown"
        self.last_state = None
        
        # Timer for periodic sensor readings (high frequency)
        self.sensor_timer = self.create_timer(0.02, self.read_sensor)  # 50Hz
        
        self.get_logger().info('Mega5 Sensor node started. Polling at 50Hz.')
    
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
        command = msg.data
        if command == "READ":
            self.read_sensor()
        else:
            self.get_logger().warn(f'Unknown sensor command: {command}')

    def read_sensor(self):
        """Read sensor state and publish"""
        response = self.send_udp_command("SENSOR")
        
        if response:
            self.last_response = response
            
            # Parse sensor state 
            if "HIGH" in response:
                sensor_state = True
            elif "LOW" in response:
                sensor_state = False
            else:
                sensor_state = self.last_state  # Keep previous state if unclear
            
            # Only publish if state changed or first reading
            if sensor_state != self.last_state or self.last_state is None:
                self.last_state = sensor_state
                
                # Publish response string
                response_msg = String()
                response_msg.data = response
                self.response_publisher.publish(response_msg)
                
                # Publish boolean state
                state_msg = Bool()
                state_msg.data = sensor_state
                self.state_publisher.publish(state_msg)
                
                self.get_logger().debug(f'Sensor response: {response}, State: {sensor_state}')

def main(args=None):
    rclpy.init(args=args)
    node = Mega5SensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
