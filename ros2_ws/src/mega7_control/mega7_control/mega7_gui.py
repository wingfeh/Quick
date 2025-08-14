#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import tkinter as tk

class Mega7GUI(Node):
    def __init__(self):
        super().__init__('mega7_gui_node')
        self.relay_publisher = self.create_publisher(Bool, 'mega7/relay_command', 10)
        self.relay_status_subscriber = self.create_subscription(Bool, 'mega7/relay_status', self.update_relay_status, 10)
        self.sensor_status_subscriber = self.create_subscription(String, 'mega7/sensor_response', self.update_sensor_status, 10)

        self.root = tk.Tk()
        self.root.title("Mega7 Control Panel")
        self.root.geometry("300x200")

        self.relay_status_label = tk.Label(self.root, text="Relay Status: Unknown", font=("Helvetica", 12))
        self.relay_status_label.pack(pady=10)

        self.sensor_status_label = tk.Label(self.root, text="Sensor Status: Unknown", font=("Helvetica", 12))
        self.sensor_status_label.pack(pady=10)

        self.on_button = tk.Button(self.root, text="Turn Relay ON", command=self.turn_on, bg="green", fg="white")
        self.on_button.pack(fill=tk.X, padx=20, pady=5)

        self.off_button = tk.Button(self.root, text="Turn Relay OFF", command=self.turn_off, bg="red", fg="white")
        self.off_button.pack(fill=tk.X, padx=20, pady=5)
        
        self.timer = self.root.after(100, self.spin_ros)

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(100, self.spin_ros)

    def turn_on(self):
        msg = Bool()
        msg.data = True
        self.relay_publisher.publish(msg)

    def turn_off(self):
        msg = Bool()
        msg.data = False
        self.relay_publisher.publish(msg)

    def update_relay_status(self, msg):
        status_text = "ON" if msg.data else "OFF"
        self.relay_status_label.config(text=f"Relay Status: {status_text}")

    def update_sensor_status(self, msg):
        status_text = "HIGH" if msg.data else "LOW"
        self.sensor_status_label.config(text=f"Sensor Status: {status_text}")

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    
    gui = Mega7GUI()
    
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
