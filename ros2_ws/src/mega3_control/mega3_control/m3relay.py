import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket

class RelayNode(Node):
    def __init__(self):
        super().__init__('m3relay_node')
        # Subscription for relay commands
        self.subscription = self.create_subscription(
            String,
            'mega3/relay_command',
            self.relay_control_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'mega3/relay_status', 10)
        self.arduino_ip = '192.168.100.103'
        self.arduino_port = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Mega3 Relay node started.')

    def send_command(self, command):
        try:
            self.sock.sendto(command.encode(), (self.arduino_ip, self.arduino_port))
            # All commands now expect responses for consistency with mega1/mega2
            data, _ = self.sock.recvfrom(1024)
            return data.decode()
        except socket.timeout:
            self.get_logger().error('Socket timeout for command: %s' % command)
        except Exception as e:
            self.get_logger().error('Socket error: %r' % e)
        return None

    def relay_control_callback(self, msg):
        """Handle incoming relay commands"""
        command = msg.data.upper()
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'ON':
            response = self.send_command('RELAY_ON')
            if response:
                self.get_logger().info(f'Arduino response: {response}')
        elif command == 'OFF':
            response = self.send_command('RELAY_OFF')
            if response:
                self.get_logger().info(f'Arduino response: {response}')
        elif command == 'STATUS':
            pass  # Status will be published by timer
        else:
            self.get_logger().warning(f'Unknown command: {command}')
        
        self.publish_status()

    def publish_status(self):
        status_response = self.send_command('STATUS')
        msg = String()
        if status_response and 'STATUS:' in status_response:
            status = status_response.split(':')[1]
            msg.data = f'Relay is {status}'
            self.publisher.publish(msg)
        else:
            msg.data = 'Relay status: Unknown (No response)'
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    relay_node = RelayNode()
    rclpy.spin(relay_node)
    relay_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
