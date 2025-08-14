#!/usr/bin/env python3
"""
Performance Test Script for Quick7 - Octuple Arduino Mega System
Tests concurrent sensor monitoring and relay activation response times across all 8 systems
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
import threading
import statistics
from collections import defaultdict, deque
import sys

class PerformanceTestNode(Node):
    def __init__(self):
        super().__init__('performance_test_node')
        
        # System identifiers
        self.megas = ['mega1', 'mega2', 'mega3', 'mega4', 'mega5', 'mega6', 'mega7', 'mega8']
        
        # Performance tracking
        self.relay_response_times = defaultdict(list)
        self.sensor_update_rates = defaultdict(lambda: deque(maxlen=100))  # Last 100 updates
        self.sensor_last_update = defaultdict(float)
        self.relay_command_start = defaultdict(float)
        
        # Publishers for relay commands
        self.relay_publishers = {}
        for mega in self.megas:
            self.relay_publishers[mega] = self.create_publisher(
                String, f'{mega}/relay_command', 10
            )
        
        # Subscribers for relay status (to measure response time)
        self.relay_status_subscribers = {}
        for mega in self.megas:
            self.relay_status_subscribers[mega] = self.create_subscription(
                String, f'{mega}/relay_status', 
                lambda msg, m=mega: self.relay_status_callback(msg, m), 10
            )
        
        # Subscribers for sensor data (to monitor update rates)
        self.sensor_subscribers = {}
        for mega in self.megas:
            # Subscribe to both sensor_response and sensor_state
            self.sensor_subscribers[f'{mega}_response'] = self.create_subscription(
                String, f'{mega}/sensor_response',
                lambda msg, m=mega: self.sensor_response_callback(msg, m), 10
            )
            self.sensor_subscribers[f'{mega}_state'] = self.create_subscription(
                Bool, f'{mega}/sensor_state',
                lambda msg, m=mega: self.sensor_state_callback(msg, m), 10
            )
        
        # Test control
        self.test_running = False
        self.concurrent_test_active = False
        
        self.get_logger().info('Performance Test Node initialized')
        self.get_logger().info('Available tests: relay_speed, sensor_monitoring, concurrent_test, full_stress')
        
    def relay_status_callback(self, msg, mega_id):
        """Track relay response times"""
        if mega_id in self.relay_command_start:
            response_time = time.time() - self.relay_command_start[mega_id]
            self.relay_response_times[mega_id].append(response_time)
            self.get_logger().info(f'{mega_id.upper()} relay response: {response_time*1000:.2f}ms - {msg.data}')
            del self.relay_command_start[mega_id]
    
    def sensor_response_callback(self, msg, mega_id):
        """Track sensor update rates from response topic"""
        current_time = time.time()
        if mega_id in self.sensor_last_update:
            interval = current_time - self.sensor_last_update[mega_id]
            self.sensor_update_rates[f'{mega_id}_response'].append(interval)
        self.sensor_last_update[mega_id] = current_time
    
    def sensor_state_callback(self, msg, mega_id):
        """Track sensor state changes"""
        current_time = time.time()
        self.get_logger().debug(f'{mega_id.upper()} sensor state: {msg.data}')
    
    def send_relay_command(self, mega_id, command):
        """Send relay command and start timing"""
        self.relay_command_start[mega_id] = time.time()
        msg = String()
        msg.data = command
        self.relay_publishers[mega_id].publish(msg)
        self.get_logger().info(f'Sent {command} to {mega_id.upper()}')
    
    def test_relay_speed(self, cycles=5):
        """Test individual relay response times"""
        self.get_logger().info(f'=== RELAY SPEED TEST - {cycles} cycles ===')
        
        for cycle in range(cycles):
            self.get_logger().info(f'--- Cycle {cycle + 1}/{cycles} ---')
            
            # Test each mega individually
            for mega in self.megas:
                # Turn ON
                self.send_relay_command(mega, 'ON')
                time.sleep(0.8)  # Wait for response
                
                # Turn OFF  
                self.send_relay_command(mega, 'OFF')
                time.sleep(0.8)  # Wait for response
                
                # Get status
                self.send_relay_command(mega, 'STATUS')
                time.sleep(0.8)  # Wait for response
            
            if cycle < cycles - 1:
                time.sleep(1.0)  # Pause between cycles
        
        # Print results
        self.print_relay_statistics()
    
    def test_concurrent_relays(self, cycles=3):
        """Test concurrent relay activation"""
        self.get_logger().info(f'=== CONCURRENT RELAY TEST - {cycles} cycles ===')
        
        for cycle in range(cycles):
            self.get_logger().info(f'--- Concurrent Cycle {cycle + 1}/{cycles} ---')
            
            # Turn all relays ON simultaneously
            self.get_logger().info('Activating ALL relays simultaneously...')
            for mega in self.megas:
                self.send_relay_command(mega, 'ON')
            time.sleep(2.0)
            
            # Turn all relays OFF simultaneously
            self.get_logger().info('Deactivating ALL relays simultaneously...')
            for mega in self.megas:
                self.send_relay_command(mega, 'OFF')
            time.sleep(2.0)
            
            if cycle < cycles - 1:
                time.sleep(1.0)
        
        self.print_relay_statistics()
    
    def monitor_sensors(self, duration=30):
        """Monitor sensor update rates for specified duration"""
        self.get_logger().info(f'=== SENSOR MONITORING TEST - {duration}s ===')
        self.get_logger().info('Monitoring sensor update rates at 80Hz...')
        
        start_time = time.time()
        
        # Clear previous data
        for mega in self.megas:
            self.sensor_update_rates[f'{mega}_response'].clear()
            if mega in self.sensor_last_update:
                del self.sensor_last_update[mega]
        
        # Monitor for specified duration
        while (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            self.get_logger().info(f'Monitoring... {elapsed:.1f}s / {duration}s')
            time.sleep(5.0)
        
        # Print sensor statistics
        self.print_sensor_statistics()
    
    def full_stress_test(self):
        """Combined stress test with concurrent operations"""
        self.get_logger().info('=== FULL STRESS TEST ===')
        
        # Start sensor monitoring in background
        self.concurrent_test_active = True
        sensor_thread = threading.Thread(target=self.background_sensor_monitor, args=(20,))
        sensor_thread.start()
        
        time.sleep(2)  # Let sensors stabilize
        
        # Perform concurrent relay operations while monitoring sensors
        self.get_logger().info('Starting concurrent relay operations...')
        
        for i in range(3):
            self.get_logger().info(f'--- Stress Cycle {i+1}/3 ---')
            
            # Rapid relay cycling
            commands = ['ON', 'OFF', 'STATUS']
            for cmd in commands:
                for mega in self.megas:
                    self.send_relay_command(mega, cmd)
                time.sleep(0.1)  # Rapid fire
            
            time.sleep(2.0)
        
        # Stop sensor monitoring
        self.concurrent_test_active = False
        sensor_thread.join()
        
        # Print comprehensive results
        self.print_comprehensive_statistics()
    
    def background_sensor_monitor(self, duration):
        """Background sensor monitoring for stress test"""
        start_time = time.time()
        while self.concurrent_test_active and (time.time() - start_time) < duration:
            time.sleep(1.0)
    
    def print_relay_statistics(self):
        """Print relay performance statistics"""
        self.get_logger().info('\n=== RELAY PERFORMANCE STATISTICS ===')
        
        for mega in self.megas:
            if self.relay_response_times[mega]:
                times_ms = [t * 1000 for t in self.relay_response_times[mega]]
                avg_time = statistics.mean(times_ms)
                min_time = min(times_ms)
                max_time = max(times_ms)
                
                self.get_logger().info(
                    f'{mega.upper()}: {len(times_ms)} responses | '
                    f'Avg: {avg_time:.2f}ms | Min: {min_time:.2f}ms | Max: {max_time:.2f}ms'
                )
            else:
                self.get_logger().warn(f'{mega.upper()}: No responses recorded')
    
    def print_sensor_statistics(self):
        """Print sensor monitoring statistics"""
        self.get_logger().info('\n=== SENSOR MONITORING STATISTICS ===')
        
        for mega in self.megas:
            response_key = f'{mega}_response'
            if self.sensor_update_rates[response_key]:
                intervals = list(self.sensor_update_rates[response_key])
                avg_interval = statistics.mean(intervals)
                frequency = 1.0 / avg_interval if avg_interval > 0 else 0
                
                self.get_logger().info(
                    f'{mega.upper()}: {len(intervals)} updates | '
                    f'Avg interval: {avg_interval*1000:.2f}ms | '
                    f'Frequency: {frequency:.1f}Hz'
                )
            else:
                self.get_logger().warn(f'{mega.upper()}: No sensor updates recorded')
    
    def print_comprehensive_statistics(self):
        """Print comprehensive test results"""
        self.get_logger().info('\n=== COMPREHENSIVE TEST RESULTS ===')
        self.print_relay_statistics()
        self.print_sensor_statistics()
        
        # Overall system performance
        total_relay_responses = sum(len(times) for times in self.relay_response_times.values())
        if total_relay_responses > 0:
            all_times_ms = []
            for times in self.relay_response_times.values():
                all_times_ms.extend([t * 1000 for t in times])
            
            overall_avg = statistics.mean(all_times_ms)
            self.get_logger().info(f'\nOVERALL SYSTEM: {total_relay_responses} total responses | Avg: {overall_avg:.2f}ms')


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: python3 performance_test.py <test_type>")
        print("Test types: relay_speed, concurrent_relays, sensor_monitoring, full_stress")
        return
    
    test_type = sys.argv[1]
    
    try:
        test_node = PerformanceTestNode()
        
        # Give time for subscriptions to establish
        time.sleep(2.0)
        
        if test_type == "relay_speed":
            test_node.test_relay_speed(cycles=5)
        elif test_type == "concurrent_relays":
            test_node.test_concurrent_relays(cycles=3)
        elif test_type == "sensor_monitoring":
            test_node.monitor_sensors(duration=30)
        elif test_type == "full_stress":
            test_node.full_stress_test()
        else:
            test_node.get_logger().error(f"Unknown test type: {test_type}")
            return
        
        test_node.get_logger().info('Test completed!')
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
