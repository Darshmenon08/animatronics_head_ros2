#!/usr/bin/env python3
"""
Motor Controller Node for Animatronics Head
Controls all 25 motors: jaw, lips, cheeks, tongue, eyes, eyebrows, and nose
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Define all motor names grouped by function
        self.motor_groups = {
            'jaw': ['jaw'],
            'lips': ['lip_up_1', 'lip_up_2', 'lip_up_3', 'lip_down_1', 'lip_down_2', 'lip_down_3'],
            'cheeks': ['left_cheek_down', 'left_cheek_up', 'right_cheek_down', 'right_cheek_up'],
            'tongue': ['tongue_in_out', 'tongue_left_right'],
            'eyes': ['left_v', 'left_h', 'right_v', 'right_h', 'left_lid', 'right_lid'],
            'eyebrows': ['left_brow_1', 'left_brow_2', 'right_brow_1', 'right_brow_2'],
            'nose': ['nose_left', 'nose_right'],
        }
        
        # Flatten to get all motor names
        self.all_motors = []
        for group in self.motor_groups.values():
            self.all_motors.extend(group)
        
        # Declare parameters for all motors with default limits
        # Motor ID mapping:
        # ID 1: left_eye_v, ID 2: left_eye_h, ID 3: right_eye_v, ID 4: right_eye_h
        # ID 5: left_lid, ID 6: right_lid
        # ID 7-10: eyebrows, ID 11-12: nose
        # ID 13-15: lip_up, ID 16-18: lip_down
        # ID 19-22: cheeks, ID 23-24: tongue, ID 25: jaw
        default_limits = {
            # Eyes (IDs 1-6)
            'left_v': [2000, 2300],          # ID 1
            'left_h': [1436, 2000],          # ID 2
            'right_v': [59, 370],            # ID 3
            'right_h': [1950, 2459],         # ID 4
            'left_lid': [2058, 2268],        # ID 5
            'right_lid': [938, 1172],        # ID 6
            # Eyebrows (IDs 7-10)
            'left_brow_1': [2040, 2414],     # ID 7
            'left_brow_2': [1412, 1777],     # ID 8
            'right_brow_1': [1616, 1907],    # ID 9
            'right_brow_2': [1440, 1734],    # ID 10
            # Nose (IDs 11-12)
            'nose_left': [1743, 2685],       # ID 11
            'nose_right': [1722, 2620],      # ID 12
            # Lips Upper (IDs 13-15)
            'lip_up_1': [1725, 2680],        # ID 13
            'lip_up_2': [1722, 2620],        # ID 14
            'lip_up_3': [1325, 2680],        # ID 15
            # Lips Lower (IDs 16-18)
            'lip_down_1': [1497, 2209],      # ID 16
            'lip_down_2': [955, 1940],       # ID 17
            'lip_down_3': [2010, 2836],      # ID 18
            # Cheeks (IDs 19-22)
            'left_cheek_down': [1060, 2413], # ID 19
            'left_cheek_up': [1555, 2230],   # ID 20
            'right_cheek_down': [1923, 3045],# ID 21
            'right_cheek_up': [2037, 2827],  # ID 22
            # Tongue (IDs 23-24)
            'tongue_in_out': [1740, 3305],   # ID 23
            'tongue_left_right': [2200, 2826],# ID 24
            # Jaw (ID 25)
            'jaw': [2000, 2300],             # ID 25
        }
        
        # Declare all motor limit parameters
        params = [(f'motor_limits.{name}', default_limits[name]) for name in self.all_motors]
        self.declare_parameters(namespace='', parameters=params)

        # Get parameters and store limits
        self.motor_limits = {}
        for name in self.all_motors:
            self.motor_limits[name] = self.get_parameter(f'motor_limits.{name}').value
            self.get_logger().info(f"Loaded limit for {name}: {self.motor_limits[name]}")

        # === Subscribers ===
        # Mouth commands (jaw, lips, cheeks, tongue)
        self.mouth_sub = self.create_subscription(
            JointTrajectory,
            '/mouth/command',
            self.mouth_command_callback,
            10)
        
        # Eye commands
        self.eye_sub = self.create_subscription(
            JointTrajectory,
            '/eye/command',
            self.eye_command_callback,
            10)
        
        # Eyebrow and nose commands
        self.brow_nose_sub = self.create_subscription(
            JointTrajectory,
            '/nose_eye_brow/command',
            self.brow_nose_command_callback,
            10)
        
        # Full face commands (all motors)
        self.face_sub = self.create_subscription(
            JointTrajectory,
            '/face/command',
            self.face_command_callback,
            10)

        # === Publishers ===
        # Publish to actual hardware drivers
        self.mouth_pub = self.create_publisher(JointTrajectory, '/mouth/joint_trajectory', 10)
        self.jaw_pub = self.create_publisher(JointTrajectory, '/jaw/joint_trajectory', 10)
        self.eye_pub = self.create_publisher(JointTrajectory, '/eye/joint_trajectory', 10)
        self.brow_nose_pub = self.create_publisher(JointTrajectory, '/nose_eye_brow/joint_trajectory', 10)
            
        self.get_logger().info('Motor Controller Node initialized with 25 motors.')

    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max."""
        return max(min_val, min(value, max_val))

    def process_trajectory(self, msg):
        """Process a JointTrajectory message and apply safety limits."""
        safe_msg = JointTrajectory()
        safe_msg.header = msg.header
        safe_msg.joint_names = msg.joint_names
        
        for point in msg.points:
            safe_point = JointTrajectoryPoint()
            safe_point.time_from_start = point.time_from_start
            safe_positions = []
            
            for i, joint_name in enumerate(msg.joint_names):
                if i < len(point.positions):
                    pos = point.positions[i]
                    
                    if joint_name in self.motor_limits:
                        limits = self.motor_limits[joint_name]
                        min_l = min(limits[0], limits[1])
                        max_l = max(limits[0], limits[1])
                        safe_pos = self.clamp(pos, min_l, max_l)
                        
                        if safe_pos != pos:
                            self.get_logger().warn(f"Clamped {joint_name}: {pos} -> {safe_pos}")
                        safe_positions.append(safe_pos)
                    else:
                        self.get_logger().warn(f"Unknown joint: {joint_name}, passing through")
                        safe_positions.append(pos)
                else:
                    self.get_logger().error(f"Position missing for joint {joint_name}")
            
            safe_point.positions = safe_positions
            safe_msg.points.append(safe_point)
            
        return safe_msg

    def mouth_command_callback(self, msg):
        """Handle mouth commands (jaw, lips, cheeks, tongue)."""
        safe_msg = self.process_trajectory(msg)
        
        # Separate jaw from other mouth motors
        jaw_msg = JointTrajectory()
        jaw_msg.header = safe_msg.header
        mouth_msg = JointTrajectory()
        mouth_msg.header = safe_msg.header
        
        jaw_names = []
        jaw_positions_list = []
        mouth_names = []
        mouth_positions_list = []
        
        for i, name in enumerate(safe_msg.joint_names):
            if name == 'jaw':
                jaw_names.append(name)
            else:
                mouth_names.append(name)
        
        jaw_msg.joint_names = jaw_names
        mouth_msg.joint_names = mouth_names
        
        for point in safe_msg.points:
            jaw_point = JointTrajectoryPoint()
            jaw_point.time_from_start = point.time_from_start
            mouth_point = JointTrajectoryPoint()
            mouth_point.time_from_start = point.time_from_start
            
            jaw_pos = []
            mouth_pos = []
            
            for i, name in enumerate(safe_msg.joint_names):
                if name == 'jaw':
                    jaw_pos.append(point.positions[i])
                else:
                    mouth_pos.append(point.positions[i])
            
            jaw_point.positions = jaw_pos
            mouth_point.positions = mouth_pos
            
            if jaw_pos:
                jaw_msg.points.append(jaw_point)
            if mouth_pos:
                mouth_msg.points.append(mouth_point)
        
        if jaw_msg.joint_names:
            self.jaw_pub.publish(jaw_msg)
        if mouth_msg.joint_names:
            self.mouth_pub.publish(mouth_msg)

    def eye_command_callback(self, msg):
        """Handle eye commands."""
        safe_msg = self.process_trajectory(msg)
        self.eye_pub.publish(safe_msg)

    def brow_nose_command_callback(self, msg):
        """Handle eyebrow and nose commands."""
        safe_msg = self.process_trajectory(msg)
        self.brow_nose_pub.publish(safe_msg)

    def face_command_callback(self, msg):
        """Handle full face commands - routes to appropriate publishers."""
        safe_msg = self.process_trajectory(msg)
        
        # Split by motor group
        groups = {
            'jaw': ([], []),
            'mouth': ([], []),
            'eye': ([], []),
            'brow_nose': ([], []),
        }
        
        mouth_motors = self.motor_groups['lips'] + self.motor_groups['cheeks'] + self.motor_groups['tongue']
        eye_motors = self.motor_groups['eyes']
        brow_nose_motors = self.motor_groups['eyebrows'] + self.motor_groups['nose']
        
        for i, name in enumerate(safe_msg.joint_names):
            if name == 'jaw':
                groups['jaw'][0].append(name)
                groups['jaw'][1].append(i)
            elif name in mouth_motors:
                groups['mouth'][0].append(name)
                groups['mouth'][1].append(i)
            elif name in eye_motors:
                groups['eye'][0].append(name)
                groups['eye'][1].append(i)
            elif name in brow_nose_motors:
                groups['brow_nose'][0].append(name)
                groups['brow_nose'][1].append(i)
        
        # Publish to each group
        publishers = {
            'jaw': self.jaw_pub,
            'mouth': self.mouth_pub,
            'eye': self.eye_pub,
            'brow_nose': self.brow_nose_pub,
        }
        
        for group_name, (names, indices) in groups.items():
            if names:
                group_msg = JointTrajectory()
                group_msg.header = safe_msg.header
                group_msg.joint_names = names
                
                for point in safe_msg.points:
                    group_point = JointTrajectoryPoint()
                    group_point.time_from_start = point.time_from_start
                    group_point.positions = [point.positions[i] for i in indices]
                    group_msg.points.append(group_point)
                
                publishers[group_name].publish(group_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
