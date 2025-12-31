#!/usr/bin/env python3
"""
Data Collector Node for Animatronics Head
Records face features and motor positions to a CSV file for training.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import csv
import os
import time
from datetime import datetime
import math

class DataCollectorNode(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # Parameters
        self.declare_parameter('output_dir', 'data')
        self.output_dir = self.get_parameter('output_dir').value
        
        # Create output directory if it doesn't exist
        # We'll use a relative path from the package or a fixed path in home
        self.base_path = os.path.expanduser(f'~/animatronics_head_ros2/{self.output_dir}')
        os.makedirs(self.base_path, exist_ok=True)
        
        # Create CSV file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = os.path.join(self.base_path, f'training_data_{timestamp}.csv')
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Define CSV headers
        self.feature_headers = [
            'eye_l_h', 'eye_l_v', 'eye_r_h', 'eye_r_v',  # Eyes (4)
            'lid_l', 'lid_r',                            # Lids (2)
            'brow_l_in', 'brow_l_out', 'brow_r_in', 'brow_r_out', # Brows (4)
            'mouth_v', 'mouth_l_h', 'mouth_r_h', 'mouth_l_v', 'mouth_r_v' # Mouth (5)
        ]
        # Total features: 15
        
        self.motor_names = [
            "left_lid", "right_lid", "left_h", "left_v", "right_h", "right_v",
            "left_brow_1", "left_brow_2", "right_brow_1", "right_brow_2",
            "lip_up_1", "lip_up_2", "lip_up_3",
            "lip_down_1", "lip_down_2", "lip_down_3",
            "left_cheek_down", "left_cheek_up",
            "right_cheek_down", "right_cheek_up",
            "jaw"
        ]
        
        headers = ['timestamp'] + self.feature_headers + self.motor_names
        self.csv_writer.writerow(headers)
        
        # State variables
        self.latest_features = None
        self.latest_motors = {}
        
        # Subscriptions
        self.create_subscription(Float32MultiArray, '/face_features', self.features_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        # Timer for recording data (10 Hz)
        self.create_timer(0.1, self.record_data)
        
        self.get_logger().info(f"Data Collector started. Recording to {self.csv_file_path}")

    def features_callback(self, msg):
        if len(msg.data) == 15:
            self.latest_features = list(msg.data)
        else:
            self.get_logger().warn(f"Received features with unexpected length: {len(msg.data)}")

    def joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.motor_names:
                self.latest_motors[name] = msg.position[i]

    def record_data(self):
        if self.latest_features is None:
            return
            
        # Check if we have all motor values
        if len(self.latest_motors) < len(self.motor_names):
            # We might not have received all motor states yet
            return
            
        row = [time.time()] + self.latest_features
        
        motor_values = []
        for name in self.motor_names:
            motor_values.append(self.latest_motors.get(name, 0.0))
            
        row.extend(motor_values)
        
        self.csv_writer.writerow(row)
        # Flush periodically to ensure data is written if crashed
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
