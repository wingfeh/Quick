import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
import time

class SensorNode(Node):
    def __init__(self):
        super().__init__('m3sensor_node')
        self.response_publisher = self.create_publisher(String, 'mega3/sensor_response', 10)
        self.state_publisher = self.create_publisher(Bool, 'mega3/sensor_state', 10)
        
        # Subscriber for sensor commands
        self.command_subscription = self.create_subscription(
            String,
            'mega3/sensor_command',
            self.command_callback,
            10
        )
        
        # State tracking
        self.last_response = "Unknown"
        self.last_state = None
        
        self.arduino_ip = '192.168.100.103'
        self.arduino_port = 8888
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)  # Standardized timeout (was 0.1)
        self.timer = self.create_timer(0.02, self.read_sensor) # 50Hz
        self.get_logger().info('Mega3 Sensor node started. Polling at 50Hz.')

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
    sensor_node = SensorNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
