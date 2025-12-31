#!/usr/bin/env python3
"""
Motor Value Checker - Utility to read and display current motor positions
Useful for calibrating motor limits and debugging
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys


class MotorValueChecker(Node):
    def __init__(self):
        super().__init__('motor_value_checker')
        
        # All motor names
        self.all_motors = [
            # Jaw
            'jaw',
            # Lips
            'lip_up_1', 'lip_up_2', 'lip_up_3',
            'lip_down_1', 'lip_down_2', 'lip_down_3',
            # Cheeks
            'left_cheek_down', 'left_cheek_up',
            'right_cheek_down', 'right_cheek_up',
            # Tongue
            'tongue_in_out', 'tongue_left_right',
            # Eyes
            'left_h', 'left_v', 'right_h', 'right_v',
            'left_lid', 'right_lid',
            # Eyebrows
            'left_brow_1', 'left_brow_2', 'right_brow_1', 'right_brow_2',
            # Nose
            'nose_left', 'nose_right',
        ]
        
        self.current_positions = {}
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Timer to display values
        self.create_timer(1.0, self.display_values)
        
        self.get_logger().info('Motor Value Checker started. Waiting for joint states...')
        self.get_logger().info('Press Ctrl+C to exit')

    def joint_state_callback(self, msg):
        """Store current motor positions."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]

    def rad_to_dxl(self, rad_value):
        """Convert radians to Dynamixel position units."""
        import math
        deg = math.degrees(rad_value) + 180
        return int((deg / 360.0) * 4095)

    def display_values(self):
        """Display current motor values."""
        if not self.current_positions:
            self.get_logger().info('No motor data received yet...')
            return
        
        print("\n" + "=" * 70)
        print(" CURRENT MOTOR POSITIONS")
        print("=" * 70)
        print(f"{'Motor Name':<25} {'Radians':>12} {'DXL Units':>12}")
        print("-" * 70)
        
        # Group motors for display
        groups = {
            'JAW': ['jaw'],
            'LIPS': ['lip_up_1', 'lip_up_2', 'lip_up_3', 'lip_down_1', 'lip_down_2', 'lip_down_3'],
            'CHEEKS': ['left_cheek_down', 'left_cheek_up', 'right_cheek_down', 'right_cheek_up'],
            'TONGUE': ['tongue_in_out', 'tongue_left_right'],
            'EYES': ['left_h', 'left_v', 'right_h', 'right_v', 'left_lid', 'right_lid'],
            'EYEBROWS': ['left_brow_1', 'left_brow_2', 'right_brow_1', 'right_brow_2'],
            'NOSE': ['nose_left', 'nose_right'],
        }
        
        for group_name, motors in groups.items():
            print(f"\n[{group_name}]")
            for motor in motors:
                if motor in self.current_positions:
                    rad_val = self.current_positions[motor]
                    dxl_val = self.rad_to_dxl(rad_val)
                    print(f"  {motor:<23} {rad_val:>12.4f} {dxl_val:>12}")
                else:
                    print(f"  {motor:<23} {'N/A':>12} {'N/A':>12}")
        
        print("=" * 70)


def main(args=None):
    rclpy.init(args=args)
    node = MotorValueChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
