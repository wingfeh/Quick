#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
import time

class Mega5RelayNode(Node):
    def __init__(self):
        super().__init__('m5relay_node')
        
        # Publishers
        self.status_publisher = self.create_publisher(String, 'mega5/relay_status', 10)
        self.state_publisher = self.create_publisher(Bool, 'mega5/relay_state', 10)
        
        # Subscriber for commands (matching GUI)
        self.command_subscription = self.create_subscription(
            String,
            'mega5/relay_command',
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
        self.last_state = None
        self.last_status = "Unknown"
        
        # Timer for periodic status updates
        self.status_timer = self.create_timer(2.0, self.request_status)
        
        self.get_logger().info('Mega5 Relay node started.')
    
    def send_udp_command(self, command):
        """Send UDP command to Arduino and return response"""
        try:
            self.sock.sendto(command.encode(), (self.arduino_ip, self.arduino_port))
            data, addr = self.sock.recvfrom(1024)
            response = data.decode().strip()
            self.get_logger().debug(f'Sent: {command}, Received: {response}')
            return response
        except socket.timeout:
            self.get_logger().error(f'Timeout sending command: {command}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error sending command {command}: {e}')
            return None
    
    def command_callback(self, msg):
        """Handle incoming relay commands"""
        command = msg.data.upper()
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'ON':
            response = self.send_udp_command('RELAY_ON')
            if response:
                self.publish_status(response)
                self.publish_state(True)
        elif command == 'OFF':
            response = self.send_udp_command('RELAY_OFF')
            if response:
                self.publish_status(response)
                self.publish_state(False)
        elif command == 'STATUS':
            self.request_status()
        else:
            self.get_logger().warning(f'Unknown command: {command}')
    
    def request_status(self):
        """Request current status from Arduino"""
        response = self.send_udp_command('STATUS')
        if response:
            self.publish_status(response)
            # Parse state from status response
            if 'ON' in response:
                self.publish_state(True)
            elif 'OFF' in response:
                self.publish_state(False)
    
    def publish_status(self, status_text):
        """Publish status message"""
        if status_text != self.last_status:
            msg = String()
            msg.data = status_text
            self.status_publisher.publish(msg)
            self.last_status = status_text
            self.get_logger().debug(f'Published status: {status_text}')
    
    def publish_state(self, state):
        """Publish boolean state"""
        if state != self.last_state:
            msg = Bool()
            msg.data = state
            self.state_publisher.publish(msg)
            self.last_state = state
            self.get_logger().debug(f'Published state: {state}')

def main(args=None):
    rclpy.init(args=args)
    relay_node = Mega5RelayNode()
    
    try:
        rclpy.spin(relay_node)
    except KeyboardInterrupt:
        pass
    finally:
        relay_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
