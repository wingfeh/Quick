#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import tkinter as tk
from tkinter import ttk
import threading
import time

class Mega1GUI(Node):
    """
    Simple GUI for controlling Arduino Mega relay
    """
    
    def __init__(self):
        super().__init__('mega1_gui')
        
        # ROS2 Publishers and Subscribers
        self.command_publisher = self.create_publisher(
            String,
            'mega1/relay_command',
            10
        )
        
        self.status_subscription = self.create_subscription(
            String,
            'mega1/relay_status',
            self.status_callback,
            10
        )
        
        self.state_subscription = self.create_subscription(
            Bool,
            'mega1/relay_state',
            self.state_callback,
            10
        )
        
        # Sensor subscriptions
        self.sensor_state_subscription = self.create_subscription(
            Bool,
            'mega1/sensor_state',
            self.sensor_state_callback,
            10
        )
        
        self.sensor_response_subscription = self.create_subscription(
            String,
            'mega1/sensor_response',
            self.sensor_response_callback,
            10
        )
        
        # Sensor command publisher
        self.sensor_command_publisher = self.create_publisher(
            String,
            'mega1/sensor_command',
            10
        )
        
        # GUI variables
        self.current_status = "Unknown"
        self.current_state = False
        self.current_sensor_state = None
        self.current_sensor_response = "Unknown"
        self.last_update = "Never"
        
        # Create GUI
        self.setup_gui()
        
        self.get_logger().info('Mega1 GUI Node Started')
    
    def setup_gui(self):
        """Create the GUI interface"""
        self.root = tk.Tk()
        self.root.title("Mega1 Relay & Sensor Controller")
        self.root.geometry("450x400")
        self.root.resizable(False, False)
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="Arduino Mega1 Relay & Sensor Controller", 
                               font=('Arial', 14, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # Status display
        status_frame = ttk.LabelFrame(main_frame, text="Current Status", padding="10")
        status_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.status_label = ttk.Label(status_frame, text="Status: Unknown", 
                                     font=('Arial', 12))
        self.status_label.grid(row=0, column=0, sticky=tk.W)
        
        self.state_label = ttk.Label(status_frame, text="Relay: Unknown", 
                                    font=('Arial', 12, 'bold'))
        self.state_label.grid(row=1, column=0, sticky=tk.W)
        
        self.update_label = ttk.Label(status_frame, text="Last Update: Never", 
                                     font=('Arial', 10))
        self.update_label.grid(row=2, column=0, sticky=tk.W)
        
        # Sensor display
        sensor_frame = ttk.LabelFrame(main_frame, text="Sensor Status (Pin 8)", padding="10")
        sensor_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.sensor_state_label = ttk.Label(sensor_frame, text="Sensor: Unknown", 
                                           font=('Arial', 12, 'bold'))
        self.sensor_state_label.grid(row=0, column=0, sticky=tk.W)
        
        self.sensor_response_label = ttk.Label(sensor_frame, text="Raw: Unknown", 
                                              font=('Arial', 10))
        self.sensor_response_label.grid(row=1, column=0, sticky=tk.W)
        
        # Control buttons
        control_frame = ttk.LabelFrame(main_frame, text="Relay Control", padding="10")
        control_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.on_button = ttk.Button(control_frame, text="Turn ON", 
                                   command=self.turn_on, width=15)
        self.on_button.grid(row=0, column=0, padx=(0, 10))
        
        self.off_button = ttk.Button(control_frame, text="Turn OFF", 
                                    command=self.turn_off, width=15)
        self.off_button.grid(row=0, column=1, padx=(10, 0))
        
        self.status_button = ttk.Button(control_frame, text="Get Status", 
                                       command=self.get_status, width=32)
        self.status_button.grid(row=1, column=0, columnspan=2, pady=(10, 0))
        
        # Sensor control
        sensor_control_frame = ttk.LabelFrame(main_frame, text="Sensor Control", padding="10")
        sensor_control_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.read_sensor_button = ttk.Button(sensor_control_frame, text="Read Sensor", 
                                           command=self.read_sensor, width=32)
        self.read_sensor_button.grid(row=0, column=0, columnspan=2)
        
        # Connection info
        info_frame = ttk.LabelFrame(main_frame, text="Connection Info", padding="10")
        info_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        ttk.Label(info_frame, text="Arduino IP: 192.168.100.101:8888", 
                 font=('Arial', 10)).grid(row=0, column=0, sticky=tk.W)
        ttk.Label(info_frame, text="ROS2 Nodes: m1relay, m1sensor", 
                 font=('Arial', 10)).grid(row=1, column=0, sticky=tk.W)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        
        # Update display
        self.update_display()
    
    def turn_on(self):
        """Send relay ON command"""
        msg = String()
        msg.data = 'ON'
        self.command_publisher.publish(msg)
        self.get_logger().info('Sent command: ON')
    
    def turn_off(self):
        """Send relay OFF command"""
        msg = String()
        msg.data = 'OFF'
        self.command_publisher.publish(msg)
        self.get_logger().info('Sent command: OFF')
    
    def get_status(self):
        """Request current status"""
        msg = String()
        msg.data = 'STATUS'
        self.command_publisher.publish(msg)
        self.get_logger().info('Requested status')
    
    def read_sensor(self):
        """Request sensor reading"""
        msg = String()
        msg.data = 'READ'
        self.sensor_command_publisher.publish(msg)
        self.get_logger().info('Requested sensor reading')
    
    def status_callback(self, msg):
        """Handle status updates from ROS2"""
        self.current_status = msg.data
        self.last_update = time.strftime("%H:%M:%S")
        self.get_logger().debug(f'Status update: {msg.data}')
        # Schedule GUI update in main thread
        self.root.after(0, self.update_display)
    
    def state_callback(self, msg):
        """Handle state updates from ROS2"""
        self.current_state = msg.data
        self.last_update = time.strftime("%H:%M:%S")
        self.get_logger().debug(f'State update: {msg.data}')
        # Schedule GUI update in main thread
        self.root.after(0, self.update_display)
    
    def sensor_state_callback(self, msg):
        """Handle sensor state updates from ROS2"""
        self.current_sensor_state = msg.data
        self.last_update = time.strftime("%H:%M:%S")
        self.get_logger().debug(f'Sensor state update: {msg.data}')
        # Schedule GUI update in main thread
        self.root.after(0, self.update_display)
    
    def sensor_response_callback(self, msg):
        """Handle sensor response updates from ROS2"""
        self.current_sensor_response = msg.data
        self.last_update = time.strftime("%H:%M:%S")
        self.get_logger().debug(f'Sensor response update: {msg.data}')
        # Schedule GUI update in main thread
        self.root.after(0, self.update_display)
    
    def update_display(self):
        """Update GUI display with current status"""
        if hasattr(self, 'status_label'):
            self.status_label.config(text=f"Status: {self.current_status}")
            
            # Update state display with color coding
            state_text = "Relay: ON" if self.current_state else "Relay: OFF"
            state_color = "green" if self.current_state else "red"
            self.state_label.config(text=state_text, foreground=state_color)
            
            # Update sensor display with color coding
            if self.current_sensor_state is not None:
                sensor_text = "Sensor: HIGH" if self.current_sensor_state else "Sensor: LOW"
                sensor_color = "blue" if self.current_sensor_state else "orange"
                self.sensor_state_label.config(text=sensor_text, foreground=sensor_color)
            else:
                self.sensor_state_label.config(text="Sensor: Unknown", foreground="gray")
            
            # Update sensor raw response
            self.sensor_response_label.config(text=f"Raw: {self.current_sensor_response}")
            
            self.update_label.config(text=f"Last Update: {self.last_update}")
    
    def run_gui(self):
        """Run the GUI main loop with ROS2 integration"""
        # Use tkinter's after method to periodically process ROS2 callbacks
        self.process_ros2_callbacks()
        self.root.mainloop()
    
    def process_ros2_callbacks(self):
        """Process ROS2 callbacks in the GUI thread"""
        try:
            # Process any pending ROS2 callbacks
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception as e:
            self.get_logger().error(f'Error processing ROS2 callbacks: {e}')
        
        # Schedule next callback processing
        self.root.after(50, self.process_ros2_callbacks)  # 20 Hz update rate

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Mega1GUI()
        
        # Run GUI in main thread (tkinter requirement)
        node.run_gui()
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
