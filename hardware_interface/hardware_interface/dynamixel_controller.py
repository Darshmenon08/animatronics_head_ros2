#!/usr/bin/env python3
"""
Dynamixel Controller Node for Animatronics Head
Communicates with Dynamixel motors via USB2Dynamixel adapters
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math

try:
    from dynamixel_sdk import *
except ImportError:
    print("ERROR: dynamixel_sdk not found. Install with: sudo apt install ros-humble-dynamixel-sdk")
    raise


# Dynamixel Protocol 2.0 Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


def rad_to_pos(angle_rad):
    """Convert radians to Dynamixel position (0-4095)."""
    angle_deg = math.degrees(angle_rad) + 180
    return int((angle_deg / 360.0) * 4095)


def pos_to_rad(position):
    """Convert Dynamixel position (0-4095) to radians."""
    angle_deg = (position / 4095.0) * 360.0 - 180
    return math.radians(angle_deg)


class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')
        
        # Declare parameters
        self.declare_parameter('baudrate', BAUDRATE)
        
        # Port configurations
        self.declare_parameter('ports.eye', '/dev/eye')
        self.declare_parameter('ports.nose_eye_brow', '/dev/nose_eye_brow')
        self.declare_parameter('ports.mouth', '/dev/mouth')
        self.declare_parameter('ports.jaw', '/dev/jaw')
        
        # Motor ID mappings (defaults)
        motor_id_defaults = {
            # Eyes
            'motor_ids.left_v': 1,
            'motor_ids.left_h': 2,
            'motor_ids.right_v': 3,
            'motor_ids.right_h': 4,
            'motor_ids.left_lid': 5,
            'motor_ids.right_lid': 6,
            # Eyebrows
            'motor_ids.left_brow_1': 7,
            'motor_ids.left_brow_2': 8,
            'motor_ids.right_brow_1': 9,
            'motor_ids.right_brow_2': 10,
            # Nose
            'motor_ids.nose_left': 11,
            'motor_ids.nose_right': 12,
            # Lips
            'motor_ids.lip_up_1': 13,
            'motor_ids.lip_up_2': 14,
            'motor_ids.lip_up_3': 15,
            'motor_ids.lip_down_1': 16,
            'motor_ids.lip_down_2': 17,
            'motor_ids.lip_down_3': 18,
            # Cheeks
            'motor_ids.left_cheek_down': 19,
            'motor_ids.left_cheek_up': 20,
            'motor_ids.right_cheek_down': 21,
            'motor_ids.right_cheek_up': 22,
            # Tongue
            'motor_ids.tongue_in_out': 23,
            'motor_ids.tongue_left_right': 24,
            # Jaw
            'motor_ids.jaw': 25,
        }
        
        for param, value in motor_id_defaults.items():
            self.declare_parameter(param, value)
        
        # Motor to port mapping
        motor_port_defaults = {
            'motor_ports.left_v': 'eye',
            'motor_ports.left_h': 'eye',
            'motor_ports.right_v': 'eye',
            'motor_ports.right_h': 'eye',
            'motor_ports.left_lid': 'eye',
            'motor_ports.right_lid': 'eye',
            'motor_ports.left_brow_1': 'nose_eye_brow',
            'motor_ports.left_brow_2': 'nose_eye_brow',
            'motor_ports.right_brow_1': 'nose_eye_brow',
            'motor_ports.right_brow_2': 'nose_eye_brow',
            'motor_ports.nose_left': 'nose_eye_brow',
            'motor_ports.nose_right': 'nose_eye_brow',
            'motor_ports.lip_up_1': 'mouth',
            'motor_ports.lip_up_2': 'mouth',
            'motor_ports.lip_up_3': 'mouth',
            'motor_ports.lip_down_1': 'mouth',
            'motor_ports.lip_down_2': 'mouth',
            'motor_ports.lip_down_3': 'mouth',
            'motor_ports.left_cheek_down': 'mouth',
            'motor_ports.left_cheek_up': 'mouth',
            'motor_ports.right_cheek_down': 'mouth',
            'motor_ports.right_cheek_up': 'mouth',
            'motor_ports.tongue_in_out': 'mouth',
            'motor_ports.tongue_left_right': 'mouth',
            'motor_ports.jaw': 'jaw',
        }
        
        for param, value in motor_port_defaults.items():
            self.declare_parameter(param, value)
        
        # Get parameters
        self.baudrate = self.get_parameter('baudrate').value
        
        # Initialize port handlers and packet handlers
        self.port_handlers = {}
        self.packet_handlers = {}
        
        port_names = ['eye', 'nose_eye_brow', 'mouth', 'jaw']
        for port_name in port_names:
            port_path = self.get_parameter(f'ports.{port_name}').value
            self.port_handlers[port_name] = PortHandler(port_path)
            self.packet_handlers[port_name] = PacketHandler(PROTOCOL_VERSION)
            
            # Open port
            if self.port_handlers[port_name].openPort():
                self.get_logger().info(f"Opened port: {port_path}")
                self.port_handlers[port_name].setBaudRate(self.baudrate)
            else:
                self.get_logger().error(f"Failed to open port: {port_path}")
        
        # Build motor mappings from parameters
        self.motor_ids = {}
        self.motor_ports = {}
        
        for param in motor_id_defaults.keys():
            motor_name = param.replace('motor_ids.', '')
            self.motor_ids[motor_name] = self.get_parameter(param).value
        
        for param in motor_port_defaults.keys():
            motor_name = param.replace('motor_ports.', '')
            self.motor_ports[motor_name] = self.get_parameter(param).value
        
        # === Subscribers ===
        self.mouth_sub = self.create_subscription(
            JointTrajectory,
            '/mouth/joint_trajectory',
            lambda msg: self.trajectory_callback(msg, 'mouth'),
            10)
        
        self.jaw_sub = self.create_subscription(
            JointTrajectory,
            '/jaw/joint_trajectory',
            lambda msg: self.trajectory_callback(msg, 'jaw'),
            10)
        
        self.eye_sub = self.create_subscription(
            JointTrajectory,
            '/eye/joint_trajectory',
            lambda msg: self.trajectory_callback(msg, 'eye'),
            10)
        
        self.brow_nose_sub = self.create_subscription(
            JointTrajectory,
            '/nose_eye_brow/joint_trajectory',
            lambda msg: self.trajectory_callback(msg, 'nose_eye_brow'),
            10)
        
        # === Publishers ===
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish joint states
        self.create_timer(0.05, self.publish_joint_states)  # 20Hz
        
        # Enable torque for all motors
        for motor_name in self.motor_ids.keys():
            if self.enable_torque(motor_name, True):
                self.get_logger().info(f"Torque enabled for {motor_name}")
            else:
                self.get_logger().warn(f"Failed to enable torque for {motor_name}")
                
        self.get_logger().info('Dynamixel Controller Node initialized.')

    def enable_torque(self, motor_name, enable=True):
        """Enable or disable torque for a motor."""
        if motor_name not in self.motor_ids:
            self.get_logger().error(f"Unknown motor: {motor_name}")
            return False
        
        motor_id = self.motor_ids[motor_name]
        port_name = self.motor_ports[motor_name]
        
        port = self.port_handlers.get(port_name)
        packet = self.packet_handlers.get(port_name)
        
        if port is None or packet is None:
            return False
        
        value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        result, error = packet.write1ByteTxRx(port, motor_id, ADDR_TORQUE_ENABLE, value)
        
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Torque control failed for {motor_name}: {packet.getTxRxResult(result)}")
            return False
        
        return True

    def set_position(self, motor_name, position):
        """Set goal position for a motor (in Dynamixel units 0-4095)."""
        if motor_name not in self.motor_ids:
            self.get_logger().error(f"Unknown motor: {motor_name}")
            return False
        
        motor_id = self.motor_ids[motor_name]
        port_name = self.motor_ports[motor_name]
        
        port = self.port_handlers.get(port_name)
        packet = self.packet_handlers.get(port_name)
        
        if port is None or packet is None:
            return False
        
        position = int(position)
        result, error = packet.write4ByteTxRx(port, motor_id, ADDR_GOAL_POSITION, position)
        
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Position write failed for {motor_name}: {packet.getTxRxResult(result)}")
            return False
        
        return True

    def get_position(self, motor_name):
        """Get current position for a motor (in Dynamixel units 0-4095)."""
        if motor_name not in self.motor_ids:
            return None
        
        motor_id = self.motor_ids[motor_name]
        port_name = self.motor_ports[motor_name]
        
        port = self.port_handlers.get(port_name)
        packet = self.packet_handlers.get(port_name)
        
        if port is None or packet is None:
            return None
        
        position, result, error = packet.read4ByteTxRx(port, motor_id, ADDR_PRESENT_POSITION)
        
        if result != COMM_SUCCESS:
            return None
        
        return position

    def trajectory_callback(self, msg, expected_port):
        """Handle JointTrajectory messages."""
        if not msg.points:
            return
        
        # Use the first point (for immediate execution)
        point = msg.points[0]
        
        for i, joint_name in enumerate(msg.joint_names):
            if i < len(point.positions):
                position = point.positions[i]
                
                # Position is expected in radians, convert to Dynamixel units
                if abs(position) < 10:  # Likely in radians
                    dxl_position = rad_to_pos(position)
                else:  # Already in Dynamixel units
                    dxl_position = int(position)
                
                # Clamp to valid range
                dxl_position = max(0, min(4095, dxl_position))
                
                self.set_position(joint_name, dxl_position)

    def publish_joint_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for motor_name in self.motor_ids.keys():
            position = self.get_position(motor_name)
            if position is not None:
                msg.name.append(motor_name)
                msg.position.append(pos_to_rad(position))
        
        if msg.name:
            self.joint_state_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources."""
        # Disable torque on all motors
        for motor_name in self.motor_ids.keys():
            self.enable_torque(motor_name, False)
        
        # Close ports
        for port in self.port_handlers.values():
            port.closePort()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
