#!/usr/bin/env python3
"""        # Publishers and Subscribers
        self.relay_sub = self.create_subscription(
            Bool,
            'mega7/relay_command',
            self.relay_command_callback,
            10
        )
        
        self.status_pub = self.create_publisher(
            Bool,
            'mega7/relay_status',
            10
        )ance Mega7 Relay Control Node
Controls relay on Arduino Mega7 via UDP with optimized performance
IP: 192.168.100.107:8888
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socket
import time
import threading

class Mega7RelayNode(Node):
    def __init__(self):
        super().__init__('m7relay_node')
        
        # Arduino Mega7 configuration
        self.arduino_ip = '192.168.100.107'
        self.arduino_port = 8888
        
        # UDP socket with timeout optimization
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)  # 500ms timeout for fast response
        
        # Publishers and Subscribers
        self.relay_sub = self.create_subscription(
            Bool,
            'mega7/relay_command',
            self.relay_callback,
            10
        )
        
        self.relay_status_pub = self.create_publisher(
            Bool,
            'mega7/relay_status',
            10
        )
        
        # Status polling timer (10Hz for responsive GUI)
        self.status_timer = self.create_timer(0.1, self.poll_status)
        
        # Thread safety
        self.command_lock = threading.Lock()
        
        self.get_logger().info('Mega7 Relay Controller Node Started')
        self.get_logger().info(f'Arduino IP: {self.arduino_ip}:{self.arduino_port}')
        
        # Initialize status
        self.last_status = None
        
    def relay_callback(self, msg):
        """Handle relay command with thread safety"""
        with self.command_lock:
            command = "RELAY_ON" if msg.data else "RELAY_OFF"
            
            try:
                # Send command to Arduino
                self.sock.sendto(command.encode(), (self.arduino_ip, self.arduino_port))
                
                # Wait for confirmation
                response, addr = self.sock.recvfrom(32)
                response_str = response.decode().strip()
                
                if "OK" in response_str:
                    self.get_logger().info(f'Relay command successful: {command} -> {response_str}')
                else:
                    self.get_logger().warn(f'Unexpected response: {response_str}')
                    
            except socket.timeout:
                self.get_logger().error('Arduino Mega7 timeout - check connection')
            except Exception as e:
                self.get_logger().error(f'Communication error: {str(e)}')
    
    def poll_status(self):
        """Poll relay status at 10Hz"""
        try:
            # Send status request
            self.sock.sendto(b"STATUS", (self.arduino_ip, self.arduino_port))
            
            # Get response
            response, addr = self.sock.recvfrom(32)
            response_str = response.decode().strip()
            
            # Parse and publish status
            if response_str == "STATUS:ON":
                status = True
            elif response_str == "STATUS:OFF":
                status = False
            else:
                return  # Invalid response, don't publish
            
            # Only publish if status changed (reduce message traffic)
            if status != self.last_status:
                status_msg = Bool()
                status_msg.data = status
                self.relay_status_pub.publish(status_msg)
                self.last_status = status
                
        except socket.timeout:
            # Timeout is normal during high-frequency polling
            pass
        except Exception as e:
            self.get_logger().error(f'Status polling error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = Mega7RelayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
