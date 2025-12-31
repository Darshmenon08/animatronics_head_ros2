#!/usr/bin/env python3
"""
Motor Slider Control GUI for Animatronics Head
Provides sliders to control all 25 motors with real-time feedback.
Uses Dynamixel SDK to communicate with motors.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import tkinter as tk
from tkinter import ttk
import threading
import yaml
import os

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("ERROR: dynamixel_sdk not found. Install with: pip install dynamixel-sdk")
    raise


# Dynamixel Protocol 2.0 Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000


class MotorSliderControl(Node):
    def __init__(self):
        super().__init__('motor_slider_control')
        
        # Motor configuration - ID, name, min, max, port
        self.motors = {
            # Eyes (Port: /dev/eye, IDs: 1-6)
            'left_v':       {'id': 1,  'min': 2000, 'max': 2300, 'port': 'eye'},  # max=ABOVE, min=BELOW
            'left_h':       {'id': 2,  'min': 1436, 'max': 2000, 'port': 'eye'},  # max=RIGHT, min=LEFT
            'right_v':      {'id': 3,  'min': 59,   'max': 370,  'port': 'eye'},  # max=BELOW, min=ABOVE
            'right_h':      {'id': 4,  'min': 1950, 'max': 2459, 'port': 'eye'},  # max=RIGHT, min=LEFT
            'left_lid':     {'id': 5,  'min': 2058, 'max': 2268, 'port': 'eye'},  # max=CLOSE, min=OPEN
            'right_lid':    {'id': 6,  'min': 938,  'max': 1172, 'port': 'eye'},  # min=CLOSE, max=OPEN
            # Eyebrows (Port: /dev/nose_eye_brow, IDs: 7-10)
            'left_brow_1':  {'id': 7,  'min': 1040, 'max': 2414, 'port': 'nose_eye_brow'},  # max=DOWN, min=UP
            'left_brow_2':  {'id': 8,  'min': 1412, 'max': 1777, 'port': 'nose_eye_brow'},  # max=DOWN, min=UP
            'right_brow_1': {'id': 9,  'min': 1616, 'max': 1907, 'port': 'nose_eye_brow'},  # min=DOWN, max=UP
            'right_brow_2': {'id': 10, 'min': 1440, 'max': 1734, 'port': 'nose_eye_brow'},  # min=DOWN, max=UP
            # Nose (Port: /dev/nose_eye_brow, IDs: 11-12)
            'nose_left':    {'id': 11, 'min': 1743, 'max': 2685, 'port': 'nose_eye_brow'},
            'nose_right':   {'id': 12, 'min': 1722, 'max': 2620, 'port': 'nose_eye_brow'},
            # Lips Upper (Port: /dev/mouth, IDs: 13-15)
            'lip_up_1':     {'id': 13, 'min': 1725, 'max': 2680, 'port': 'mouth'},  # max=UP, min=DOWN
            'lip_up_2':     {'id': 14, 'min': 1722, 'max': 2620, 'port': 'mouth'},  # max=UP, min=DOWN
            'lip_up_3':     {'id': 15, 'min': 1325, 'max': 2680, 'port': 'mouth'},  # min=UP, max=DOWN
            # Lips Lower (Port: /dev/mouth, IDs: 16-18)
            'lip_down_1':   {'id': 16, 'min': 1497, 'max': 2209, 'port': 'mouth'},  # max=DOWN, min=UP
            'lip_down_2':   {'id': 17, 'min': 955,  'max': 1940, 'port': 'mouth'},  # min=DOWN, max=UP
            'lip_down_3':   {'id': 18, 'min': 2010, 'max': 2836, 'port': 'mouth'},  # max=DOWN, min=UP
            # Cheeks (Port: /dev/mouth, IDs: 19-22)
            'left_cheek_down':  {'id': 19, 'min': 1060, 'max': 2413, 'port': 'mouth'},  # max=DOWN, min=UP
            'left_cheek_up':    {'id': 20, 'min': 1555, 'max': 2230, 'port': 'mouth'},  # min=UP, max=DOWN
            'right_cheek_down': {'id': 21, 'min': 1923, 'max': 3045, 'port': 'mouth'},  # min=DOWN, max=UP
            'right_cheek_up':   {'id': 22, 'min': 2037, 'max': 2827, 'port': 'mouth'},  # max=UP, min=DOWN
            # Tongue (Port: /dev/mouth, IDs: 23-24)
            'tongue_in_out':    {'id': 23, 'min': 1740, 'max': 3305, 'port': 'mouth'},
            'tongue_left_right': {'id': 24, 'min': 2200, 'max': 2826, 'port': 'mouth'},
            # Jaw (Port: /dev/jaw, ID: 25)
            'jaw':          {'id': 25, 'min': 2100, 'max': 2200, 'port': 'jaw'},
        }
        
        # Port paths
        self.port_paths = {
            'eye': '/dev/eye',
            'nose_eye_brow': '/dev/nose_eye_brow',
            'mouth': '/dev/mouth',
            'jaw': '/dev/jaw',
        }
        
        # Initialize port handlers
        self.port_handlers = {}
        self.packet_handlers = {}
        self.ports_open = {}
        
        for port_name, port_path in self.port_paths.items():
            self.port_handlers[port_name] = PortHandler(port_path)
            self.packet_handlers[port_name] = PacketHandler(PROTOCOL_VERSION)
            
            if self.port_handlers[port_name].openPort():
                self.port_handlers[port_name].setBaudRate(BAUDRATE)
                self.ports_open[port_name] = True
                self.get_logger().info(f"Opened port: {port_path}")
            else:
                self.ports_open[port_name] = False
                self.get_logger().warn(f"Failed to open port: {port_path}")
        
        # Slider values
        self.slider_vars = {}
        self.position_labels = {}
        
        self.get_logger().info('Motor Slider Control initialized')
    
    def enable_torque(self, motor_name, enable=True):
        """Enable or disable torque for a motor."""
        motor = self.motors[motor_name]
        port_name = motor['port']
        
        if not self.ports_open.get(port_name, False):
            self.get_logger().error(f"Port {port_name} not open for {motor_name}")
            return False
        
        port = self.port_handlers[port_name]
        packet = self.packet_handlers[port_name]
        
        value = 1 if enable else 0
        result, error = packet.write1ByteTxRx(port, motor['id'], ADDR_TORQUE_ENABLE, value)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Torque enable failed for {motor_name} (ID:{motor['id']}): {packet.getTxRxResult(result)}")
        if error != 0:
            self.get_logger().error(f"Torque enable error for {motor_name}: {packet.getRxPacketError(error)}")
        return result == COMM_SUCCESS
    
    def set_position(self, motor_name, position):
        """Set motor position."""
        motor = self.motors[motor_name]
        port_name = motor['port']
        
        if not self.ports_open.get(port_name, False):
            self.get_logger().error(f"Port {port_name} not open for {motor_name}")
            return False
        
        port = self.port_handlers[port_name]
        packet = self.packet_handlers[port_name]
        
        # Clamp to limits
        position = max(motor['min'], min(motor['max'], int(position)))
        
        result, error = packet.write4ByteTxRx(port, motor['id'], ADDR_GOAL_POSITION, position)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Position set failed for {motor_name} (ID:{motor['id']}): {packet.getTxRxResult(result)}")
        if error != 0:
            self.get_logger().error(f"Position set error for {motor_name}: {packet.getRxPacketError(error)}")
        return result == COMM_SUCCESS
    
    def get_position(self, motor_name):
        """Get current motor position."""
        motor = self.motors[motor_name]
        port_name = motor['port']
        
        if not self.ports_open.get(port_name, False):
            return None
        
        port = self.port_handlers[port_name]
        packet = self.packet_handlers[port_name]
        
        position, result, _ = packet.read4ByteTxRx(port, motor['id'], ADDR_PRESENT_POSITION)
        return position if result == COMM_SUCCESS else None
    
    def on_slider_change(self, motor_name, value):
        """Called when a slider value changes."""
        position = int(float(value))
        
        # Auto-enable torque before setting position
        if not self.enable_torque(motor_name, True):
            self.get_logger().warn(f"Failed to enable torque for {motor_name}")
        
        if not self.set_position(motor_name, position):
            self.get_logger().warn(f"Failed to set position for {motor_name}")
        
        # Update label
        if motor_name in self.position_labels:
            self.position_labels[motor_name].config(text=str(position))
    
    def enable_all_torque(self):
        """Enable torque on all motors."""
        for motor_name in self.motors:
            self.enable_torque(motor_name, True)
        self.get_logger().info("Enabled torque on all motors")
    
    def disable_all_torque(self):
        """Disable torque on all motors."""
        for motor_name in self.motors:
            self.enable_torque(motor_name, False)
        self.get_logger().info("Disabled torque on all motors")
    
    def center_all(self):
        """Move all motors to center position."""
        for motor_name, motor in self.motors.items():
            center = (motor['min'] + motor['max']) // 2
            self.slider_vars[motor_name].set(center)
            self.set_position(motor_name, center)
    
    def run_gui(self):
        """Run the Tkinter GUI."""
        self.root = tk.Tk()
        self.root.title("Animatronics Head Motor Control")
        self.root.geometry("1200x800")
        
        # Create notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Group motors by category
        groups = {
            'Eyes': ['left_v', 'left_h', 'right_v', 'right_h', 'left_lid', 'right_lid'],
            'Eyebrows': ['left_brow_1', 'left_brow_2', 'right_brow_1', 'right_brow_2'],
            'Nose': ['nose_left', 'nose_right'],
            'Lips Upper': ['lip_up_1', 'lip_up_2', 'lip_up_3'],
            'Lips Lower': ['lip_down_1', 'lip_down_2', 'lip_down_3'],
            'Cheeks': ['left_cheek_down', 'left_cheek_up', 'right_cheek_down', 'right_cheek_up'],
            'Tongue': ['tongue_in_out', 'tongue_left_right'],
            'Jaw': ['jaw'],
        }
        
        for group_name, motor_names in groups.items():
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=group_name)
            
            for i, motor_name in enumerate(motor_names):
                motor = self.motors[motor_name]
                
                # Motor label
                label = ttk.Label(frame, text=f"{motor_name} (ID: {motor['id']})", width=20)
                label.grid(row=i, column=0, padx=5, pady=5, sticky='w')
                
                # Min label
                min_label = ttk.Label(frame, text=str(motor['min']), width=6)
                min_label.grid(row=i, column=1, padx=2)
                
                # Slider
                var = tk.IntVar(value=(motor['min'] + motor['max']) // 2)
                self.slider_vars[motor_name] = var
                
                slider = ttk.Scale(
                    frame,
                    from_=motor['min'],
                    to=motor['max'],
                    variable=var,
                    orient='horizontal',
                    length=400,
                    command=lambda v, m=motor_name: self.on_slider_change(m, v)
                )
                slider.grid(row=i, column=2, padx=5, pady=5)
                
                # Max label
                max_label = ttk.Label(frame, text=str(motor['max']), width=6)
                max_label.grid(row=i, column=3, padx=2)
                
                # Current position label
                pos_label = ttk.Label(frame, text=str(var.get()), width=6)
                pos_label.grid(row=i, column=4, padx=5)
                self.position_labels[motor_name] = pos_label
        
        # Control buttons frame
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill='x', padx=10, pady=10)
        
        ttk.Button(btn_frame, text="Enable All Torque", command=self.enable_all_torque).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Disable All Torque", command=self.disable_all_torque).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Center All", command=self.center_all).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Quit", command=self.quit_app).pack(side='right', padx=5)
        
        # Process ROS2 callbacks periodically
        self.root.after(100, self.ros_spin_once)
        
        self.root.mainloop()
    
    def ros_spin_once(self):
        """Process ROS2 callbacks."""
        rclpy.spin_once(self, timeout_sec=0)
        if hasattr(self, 'root') and self.root:
            self.root.after(100, self.ros_spin_once)
    
    def quit_app(self):
        """Quit the application."""
        self.running = False
        if hasattr(self, 'root') and self.root:
            self.root.quit()
            self.root.destroy()
    
    def destroy_node(self):
        """Clean up."""
        self.disable_all_torque()
        for port in self.port_handlers.values():
            port.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorSliderControl()
    node.running = True
    
    try:
        # Run GUI in main thread (blocking)
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
