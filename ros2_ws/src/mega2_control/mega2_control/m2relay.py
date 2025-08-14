#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
import time

class M2RelayController(Node):
    """
    ROS2 Node to control Arduino Mega2 relay via UDP
    """
    
    def __init__(self):
        super().__init__('m2relay_node')
        
        # Arduino connection parameters - MEGA2 uses different IP
        self.arduino_ip = '192.168.100.102'
        self.arduino_port = 8888
        self.socket_timeout = 0.5  # Standardized timeout
        
        # ROS2 Publishers and Subscribers
        self.command_subscription = self.create_subscription(
            String,
            'mega2/relay_command',
            self.relay_command_callback,
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            'mega2/relay_status',
            10
        )
        
        self.state_publisher = self.create_publisher(
            Bool,
            'mega2/relay_state',
            10
        )
        
        self.response_publisher = self.create_publisher(
            String,
            'mega2/relay_response',
            10
        )
        
        # UDP socket for Arduino communication
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(self.socket_timeout)
        
        # Response caching to eliminate redundant updates
        self.last_status = None
        self.last_state = None
        
        # Timer for periodic status updates (1Hz)
        self.status_timer = self.create_timer(1.0, self.get_status)
        
        self.get_logger().info('M2 Relay Controller Node Started - OPTIMIZED MODE')
        self.get_logger().info(f'Arduino IP: {self.arduino_ip}:{self.arduino_port}')
        
        # Test initial connection
        self.test_connection()
    
    def test_connection(self):
        """Test initial connection to Arduino"""
        try:
            response = self.send_udp_command('STATUS')
            if response:
                self.get_logger().info(f'Successfully connected to Arduino Mega2: {response}')
            else:
                self.get_logger().warn('No response from Arduino Mega2 - check connection')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino Mega2: {str(e)}')
    
    def send_udp_command(self, command):
        """Send UDP command to Arduino and wait for response"""
        try:
            # Send command
            self.socket.sendto(command.encode(), (self.arduino_ip, self.arduino_port))
            self.get_logger().debug(f'Sent command: {command}')
            
            # Wait for response
            data, addr = self.socket.recvfrom(32)  # Minimal buffer
            response = data.decode().strip()
            self.get_logger().debug(f'Received response: {response}')
            return response
            
        except socket.timeout:
            self.get_logger().warn(f'Timeout waiting for response to command: {command}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error sending command {command}: {str(e)}')
            return None
    
    def relay_command_callback(self, msg):
        """Handle relay command from ROS2"""
        command = msg.data.upper()
        self.get_logger().info(f'Received command: {command}')
        
        # Map ROS2 commands to Arduino commands
        if command == 'ON':
            arduino_command = 'RELAY_ON'
        elif command == 'OFF':
            arduino_command = 'RELAY_OFF'
        elif command == 'STATUS':
            arduino_command = 'STATUS'
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return
        
        # Send command to Arduino
        response = self.send_udp_command(arduino_command)
        
        if response:
            # Publish raw response
            response_msg = String()
            response_msg.data = response
            self.response_publisher.publish(response_msg)
            
            # Parse and publish status/state
            self.parse_and_publish_response(response)
        else:
            self.get_logger().error(f'Failed to get response for command: {command}')
    
    def get_status(self):
        """Periodic status check"""
        response = self.send_udp_command('STATUS')
        if response:
            self.parse_and_publish_response(response)
    
    def parse_and_publish_response(self, response):
        """Parse Arduino response and publish appropriate ROS2 messages"""
        # Only publish if status changed (response caching)
        if response != self.last_status:
            status_msg = String()
            status_msg.data = response
            self.status_publisher.publish(status_msg)
            self.last_status = response
            
            # Parse state from response
            new_state = None
            if 'STATUS:ON' in response or 'RELAY_ON:OK' in response:
                new_state = True
            elif 'STATUS:OFF' in response or 'RELAY_OFF:OK' in response:
                new_state = False
            
            # Only publish state if it changed
            if new_state is not None and new_state != self.last_state:
                state_msg = Bool()
                state_msg.data = new_state
                self.state_publisher.publish(state_msg)
                self.last_state = new_state
                self.get_logger().debug(f'State changed to: {new_state}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        relay_controller = M2RelayController()
        rclpy.spin(relay_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'relay_controller' in locals():
            relay_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
