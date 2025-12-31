#!/usr/bin/env python3
"""
Eye Controller Node for Animatronics Head
Uses MediaPipe face mesh to track eyes and control eye motors
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

try:
    import mediapipe as mp
except ImportError:
    print("ERROR: mediapipe not found. Install with: pip install mediapipe")
    raise


def rad(angle):
    """Convert degrees to radians."""
    return (angle - 180) * math.pi / 180


class EyeController(Node):
    def __init__(self):
        super().__init__('eye_controller')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('smoothing_window', 10)
        
        self.camera_id = self.get_parameter('camera_id').value
        self.smoothing_window = self.get_parameter('smoothing_window').value
        
        # Publisher for eye commands
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/eye/command', 
            10)
        
        # MediaPipe setup
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        
        # Smoothing buffer
        from collections import deque
        self.eyes_avg = deque(maxlen=self.smoothing_window)
        
        # Video capture
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_id}")
        
        # Timer for processing frames
        self.create_timer(1.0 / 30.0, self.process_frame)  # 30 Hz
        
        self.get_logger().info(f'Eye Controller started with camera {self.camera_id}')

    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two landmarks."""
        return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) ** 0.5

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Map a value from one range to another."""
        value = max(min(value, in_max), in_min)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def process_frame(self):
        """Process a single frame from the camera."""
        success, frame = self.cap.read()
        if not success:
            return
        
        # Convert to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(frame_rgb)
        
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # Get iris and eye landmarks
                left_iris = face_landmarks.landmark[468]
                right_iris = face_landmarks.landmark[473]
                left_eye_inner = face_landmarks.landmark[133]
                left_eye_outer = face_landmarks.landmark[33]
                right_eye_inner = face_landmarks.landmark[362]
                right_eye_outer = face_landmarks.landmark[263]
                
                # Calculate vertical bounds
                left_v = self.calculate_distance(
                    face_landmarks.landmark[470], 
                    face_landmarks.landmark[472]) / 2
                right_v = self.calculate_distance(
                    face_landmarks.landmark[475], 
                    face_landmarks.landmark[477]) / 2
                left_u = (left_eye_outer.y + left_eye_inner.y) / 2
                right_u = (right_eye_outer.y + right_eye_inner.y) / 2
                
                # Map iris position to 0-100 range
                left_horizontal = self.map_value(
                    left_iris.x, left_eye_outer.x, left_eye_inner.x, 0, 100)
                left_vertical = self.map_value(
                    left_iris.y, left_u - left_v, left_u + left_v, 0, 100)
                right_horizontal = self.map_value(
                    right_iris.x, right_eye_inner.x, right_eye_outer.x, 0, 100)
                right_vertical = self.map_value(
                    right_iris.y, right_u - right_v, right_u + right_v, 0, 100)
                
                # Add to smoothing buffer
                self.eyes_avg.append((
                    left_horizontal, left_vertical,
                    right_horizontal, right_vertical,
                    50, 50  # Default lid positions
                ))
                
                # Calculate smoothed values
                if self.eyes_avg:
                    smoothed = [
                        sum(x[i] for x in self.eyes_avg) / len(self.eyes_avg)
                        for i in range(6)
                    ]
                    self.publish_eye_trajectory(smoothed)

    def publish_eye_trajectory(self, positions):
        """Publish eye motor positions."""
        # Convert 0-100 to radians
        positions = [
            self.map_value(p, 0, 100, rad(0), rad(180))
            for p in positions
        ]
        
        msg = JointTrajectory()
        msg.joint_names = [
            'left_eye_h', 'left_eye_v',
            'right_eye_h', 'right_eye_v',
            'left_lid', 'right_lid'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1 seconds
        
        msg.points.append(point)
        self.publisher.publish(msg)

    def destroy_node(self):
        """Clean up resources."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EyeController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
