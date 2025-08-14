#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
import threading
import time

class M2SensorNode(Node):
    def __init__(self):
        super().__init__('m2sensor_node')
        
        # Publishers
        self.sensor_state_publisher = self.create_publisher(Bool, 'mega2/sensor_state', 10)
        self.sensor_response_publisher = self.create_publisher(String, 'mega2/sensor_response', 10)
        
        # Subscribers
        self.sensor_command_subscription = self.create_subscription(
            String,
            'mega2/sensor_command',
            self.sensor_command_callback,
            10
        )
        
        # Arduino UDP configuration - MEGA2 uses different IP
        self.arduino_ip = '192.168.100.102'
        self.arduino_port = 8888
        self.socket_timeout = 0.5  # Standardized timeout (was 0.1)
        
        # Initialize UDP socket
        self.setup_udp_socket()
        
        # State tracking
        self.last_sensor_state = None
        
        # Timer for periodic sensor reading (50Hz) - standardized with other megas
        self.create_timer(0.02, self.read_sensor_timer_callback)
        
        self.get_logger().info('Mega2 Sensor node started. Polling at 50Hz.')
        
    def setup_udp_socket(self):
        """Setup UDP socket for Arduino communication"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(self.socket_timeout)
            self.get_logger().info(f'UDP socket configured for {self.arduino_ip}:{self.arduino_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to setup UDP socket: {e}')
            
    def send_arduino_command(self, command):
        """Send command to Arduino and get response"""
        try:
            # Send command
            self.sock.sendto(command.encode(), (self.arduino_ip, self.arduino_port))
            
            # Receive response
            response, addr = self.sock.recvfrom(32)
            return response.decode().strip()
            
        except socket.timeout:
            self.get_logger().warn(f'Arduino timeout for command: {command}')
            return None
        except Exception as e:
            self.get_logger().error(f'Arduino communication error: {e}')
            return None
            
    def sensor_command_callback(self, msg):
        """Handle sensor command requests"""
        command = msg.data.upper()
        
        if command == 'READ':
            # Read sensor directly
            response = self.send_arduino_command('SENSOR')
            
            if response:
                # Publish raw response
                response_msg = String()
                response_msg.data = response
                self.sensor_response_publisher.publish(response_msg)
                
                # Parse and publish boolean state
                if 'HIGH' in response:
                    sensor_state = Bool()
                    sensor_state.data = True
                    self.sensor_state_publisher.publish(sensor_state)
                    self.get_logger().info('Sensor: HIGH')
                elif 'LOW' in response:
                    sensor_state = Bool()
                    sensor_state.data = False
                    self.sensor_state_publisher.publish(sensor_state)
                    self.get_logger().info('Sensor: LOW')
                    
        else:
            self.get_logger().warn(f'Unknown sensor command: {command}')
            
    def read_sensor_timer_callback(self):
        """Periodic sensor reading (10Hz)"""
        response = self.send_arduino_command('SENSOR')
        
        if response:
            # Parse sensor state
            current_state = None
            if 'HIGH' in response:
                current_state = True
            elif 'LOW' in response:
                current_state = False
                
            # Only publish if state changed (reduce traffic)
            if current_state is not None and current_state != self.last_sensor_state:
                sensor_state = Bool()
                sensor_state.data = current_state
                self.sensor_state_publisher.publish(sensor_state)
                
                # Also publish raw response
                response_msg = String()
                response_msg.data = response
                self.sensor_response_publisher.publish(response_msg)
                
                self.get_logger().info(f'Sensor state changed: {current_state}')
                self.last_sensor_state = current_state

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sensor_node = M2SensorNode()
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'sensor_node' in locals():
            sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
