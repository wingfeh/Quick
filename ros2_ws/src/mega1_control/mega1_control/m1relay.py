#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
import time

class M1RelayControllerOptimized(Node):
    """
    OPTIMIZED ROS2 Node for ultra-fast Arduino Mega relay control
    """
    
    def __init__(self):
        super().__init__('m1relay')
        
        # Arduino connection parameters
        self.arduino_ip = '192.168.100.101'
        self.arduino_port = 8888
        
        # ROS2 Publishers and Subscribers with smaller queue for faster processing
        self.command_subscription = self.create_subscription(
            String,
            'mega1/relay_command',
            self.relay_command_callback,
            1  # Reduced queue size for faster processing
        )
        
        self.status_publisher = self.create_publisher(
            String,
            'mega1/relay_status',
            1
        )
        
        self.state_publisher = self.create_publisher(
            Bool,
            'mega1/relay_state',
            1
        )
        
        # Pre-create socket for reuse (no repeated socket creation overhead)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(0.1)  # Very short timeout for speed
        
        # Connection cache for faster responses
        self.last_status = "UNKNOWN"
        self.last_state = False
        
        # Faster status updates (reduced from 5s to 1s)
        self.status_timer = self.create_timer(1.0, self.get_status)
        
        # Initial connection test
        self.test_connection()
        
        self.get_logger().info('M1 Relay Controller Node Started - OPTIMIZED MODE')
        self.get_logger().info(f'Arduino IP: {self.arduino_ip}:{self.arduino_port}')

    def test_connection(self):
        """Fast initial connection test"""
        try:
            response = self.send_udp_command("STATUS")
            if response:
                self.get_logger().info(f'Successfully connected to Arduino: {response}')
            else:
                self.get_logger().warn('Initial connection test failed')
        except Exception as e:
            self.get_logger().error(f'Connection test error: {e}')

    def send_udp_command(self, command):
        """
        Optimized UDP command sender with minimal overhead
        """
        try:
            # Send command (no encoding overhead - direct bytes)
            self.socket.sendto(command.encode('ascii'), (self.arduino_ip, self.arduino_port))
            
            # Receive response with minimal timeout
            data, addr = self.socket.recvfrom(32)  # Small buffer for speed
            return data.decode('ascii')
            
        except socket.timeout:
            return None
        except Exception as e:
            self.get_logger().error(f'UDP communication error: {e}')
            return None

    def relay_command_callback(self, msg):
        """
        Optimized command processing with immediate response
        """
        command = msg.data.upper()
        
        # Fast command mapping
        if command == 'ON':
            udp_command = 'RELAY_ON'
        elif command == 'OFF':
            udp_command = 'RELAY_OFF'
        elif command == 'STATUS':
            udp_command = 'STATUS'
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return
        
        # Send command and get immediate response
        response = self.send_udp_command(udp_command)
        
        if response:
            # Publish response immediately
            self.publish_status_update(response)
            self.get_logger().debug(f'Command {command} -> {response}')
        else:
            self.get_logger().error(f'No response for command: {command}')

    def publish_status_update(self, response):
        """
        Fast status publishing with caching
        """
        # Parse response for state
        if 'RELAY_ON:OK' in response or 'STATUS:ON' in response:
            state = True
            status = 'ON'
        elif 'RELAY_OFF:OK' in response or 'STATUS:OFF' in response:
            state = False
            status = 'OFF'
        else:
            status = response
            state = self.last_state
        
        # Only publish if changed (reduces network overhead)
        if status != self.last_status:
            status_msg = String()
            status_msg.data = status
            self.status_publisher.publish(status_msg)
            self.last_status = status
        
        if state != self.last_state:
            state_msg = Bool()
            state_msg.data = state
            self.state_publisher.publish(state_msg)
            self.last_state = state

    def get_status(self):
        """
        Fast periodic status check
        """
        response = self.send_udp_command("STATUS")
        if response:
            self.publish_status_update(response)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = M1RelayControllerOptimized()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
