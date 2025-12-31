#!/usr/bin/env python3
"""
Face Mimic Node for Animatronics Head (ROS2)
Uses MediaPipe face mesh to track facial landmarks and control all motors.
Ported from ROS1 mimic.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import cv2
import math
from collections import deque

try:
    import mediapipe as mp
except ImportError:
    print("ERROR: mediapipe not found. Install with: pip install mediapipe")
    raise


def rad(angle):
    """Convert a value in [0, 4095] to radians, centered at 0."""
    return (((angle * 360) / 4095) - 180) * math.pi / 180


def rad_from_deg(angle):
    """Convert degrees to radians, centered at 0."""
    return (angle - 180) * math.pi / 180


def calculate_distance(point1, point2):
    """Euclidean distance between two points with .x and .y attributes."""
    return (((point1.x - point2.x) ** 2) + ((point1.y - point2.y) ** 2)) ** 0.5


def smooth(current_value, previous_value, alpha=0.01):
    """Exponential moving average for smoothing values."""
    return alpha * current_value + (1 - alpha) * previous_value


def map_value(value, in_min, in_max, out_min, out_max):
    """Map value from one range to another, clamped to input range."""
    value = max(min(value, in_max), in_min)
    return round((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, 5)


class FaceMimicNode(Node):
    def __init__(self):
        super().__init__('face_mimic')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('show_video', True)
        self.declare_parameter('smoothing_window', 10)
        
        self.camera_id = self.get_parameter('camera_id').value
        self.show_video = self.get_parameter('show_video').value
        self.q_size = self.get_parameter('smoothing_window').value
        
        # Publishers
        self.publisher_eye = self.create_publisher(JointTrajectory, '/eye/joint_trajectory', 10)
        self.publisher_jaw = self.create_publisher(JointTrajectory, '/jaw/joint_trajectory', 10)
        self.publisher_eye_brow = self.create_publisher(JointTrajectory, '/nose_eye_brow/joint_trajectory', 10)
        self.publisher_mouth = self.create_publisher(JointTrajectory, '/mouth/joint_trajectory', 10)
        
        # MediaPipe setup
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Video capture
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at index {self.camera_id}")
            raise RuntimeError(f"Camera at index {self.camera_id} not available")
        
        self.frame_width = 640
        self.frame_height = 480
        self.face_width = 1.0
        self.face_height = 1.0
        
        # Smoothing buffers
        self.eyes_avg = deque(maxlen=self.q_size)
        self.eyes_brow_avg = deque(maxlen=self.q_size)
        self.mouth_r_h_avg = deque(maxlen=self.q_size)
        self.mouth_l_h_avg = deque(maxlen=self.q_size)
        self.mouth_v_avg = deque(maxlen=self.q_size)
        self.mouth_l_v_avg = deque(maxlen=self.q_size)
        self.mouth_r_v_avg = deque(maxlen=self.q_size)
        
        # Timer for processing frames (30 Hz)
        self.create_timer(1.0 / 30.0, self.process_frame)
        
        self.get_logger().info(f'Face Mimic Node started with camera {self.camera_id}')
    
    def draw_points(self, frame, coord):
        """Draw a point on the frame."""
        coords = (int(coord.x * self.frame_width), int(coord.y * self.frame_height))
        cv2.circle(frame, coords, 3, (0, 255, 0), -1)
    
    def viewing_mesh(self, frame, face_landmarks):
        """Draw face mesh on frame."""
        self.mp_drawing.draw_landmarks(
            image=frame,
            landmark_list=face_landmarks,
            connections=self.mp_face_mesh.FACEMESH_TESSELATION,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_tesselation_style()
        )
        self.mp_drawing.draw_landmarks(
            image=frame,
            landmark_list=face_landmarks,
            connections=self.mp_face_mesh.FACEMESH_CONTOURS,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style()
        )
        self.mp_drawing.draw_landmarks(
            image=frame,
            landmark_list=face_landmarks,
            connections=self.mp_face_mesh.FACEMESH_IRISES,
            landmark_drawing_spec=None,
            connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_iris_connections_style()
        )
    
    def safe_average(self, values):
        """Safely calculate average of values."""
        if not values:
            return 0.0
        return sum(values) / len(values)
    
    def safe_smooth(self, current_value, previous_values, alpha=0.01):
        """Safely apply smoothing."""
        if not previous_values:
            return current_value
        return smooth(current_value, previous_values[-1], alpha)
    
    def calculate_eyebrow_positions(self, face_landmarks):
        """Calculate eyebrow positions using multiple landmarks."""
        left_brow_landmarks = [
            face_landmarks.landmark[107],  # Inner
            face_landmarks.landmark[66],   # Inner-mid
            face_landmarks.landmark[105],  # Mid
            face_landmarks.landmark[63],   # Outer-mid
            face_landmarks.landmark[70]    # Outer
        ]
        
        right_brow_landmarks = [
            face_landmarks.landmark[336],  # Inner
            face_landmarks.landmark[296],  # Inner-mid
            face_landmarks.landmark[334],  # Mid
            face_landmarks.landmark[293],  # Outer-mid
            face_landmarks.landmark[300]   # Outer
        ]
        
        forehead_center = face_landmarks.landmark[10]
        
        left_brow_heights = []
        right_brow_heights = []
        
        for landmark in left_brow_landmarks:
            height = (forehead_center.y - landmark.y) / self.face_height
            left_brow_heights.append(height)
        
        for landmark in right_brow_landmarks:
            height = (forehead_center.y - landmark.y) / self.face_height
            right_brow_heights.append(height)
        
        # Calculate final positions
        left_inner = (left_brow_heights[0] + 0.1) * 100
        left_outer = (left_brow_heights[-1] + 0.1) * 100
        right_inner = (right_brow_heights[0] + 0.1) * 100
        right_outer = (right_brow_heights[-1] + 0.1) * 100
        
        # Clamp to reasonable range
        left_inner = max(20, min(80, left_inner))
        left_outer = max(20, min(80, left_outer))
        right_inner = max(20, min(80, right_inner))
        right_outer = max(20, min(80, right_outer))
        
        return [left_inner, left_outer, right_inner, right_outer]
    
    def process_frame(self):
        """Process a single frame from the camera."""
        success, frame = self.cap.read()
        if not success:
            return
        
        frame.flags.writeable = False
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(frame_rgb)
        
        frame.flags.writeable = True
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                try:
                    self.frame_height, self.frame_width, _ = frame.shape
                    
                    if self.show_video:
                        self.viewing_mesh(frame, face_landmarks)
                    
                    # Face dimensions
                    self.face_width = abs(face_landmarks.landmark[361].x - face_landmarks.landmark[133].x)
                    self.face_height = abs(face_landmarks.landmark[10].y - face_landmarks.landmark[152].y)
                    
                    # === MOUTH PROCESSING ===
                    upper_lip = face_landmarks.landmark[13]
                    lower_lip = face_landmarks.landmark[14]
                    upper_left_lip = face_landmarks.landmark[311]
                    lower_left_lip = face_landmarks.landmark[402]
                    upper_right_lip = face_landmarks.landmark[81]
                    lower_right_lip = face_landmarks.landmark[178]
                    left_lip = face_landmarks.landmark[308]
                    right_lip = face_landmarks.landmark[62]
                    
                    class MidLip:
                        pass
                    mid_lip = MidLip()
                    mid_lip.x = (left_lip.x + right_lip.x) / 2
                    mid_lip.y = (left_lip.y + right_lip.y) / 2
                    
                    self.mouth_l_h_avg.append(calculate_distance(left_lip, mid_lip) * 2 / self.face_width)
                    self.mouth_r_h_avg.append(calculate_distance(mid_lip, right_lip) * 2 / self.face_width)
                    self.mouth_l_v_avg.append(calculate_distance(upper_left_lip, lower_left_lip) / self.face_height)
                    self.mouth_v_avg.append(calculate_distance(upper_lip, lower_lip) / self.face_height)
                    self.mouth_r_v_avg.append(calculate_distance(upper_right_lip, lower_right_lip) / self.face_height)
                    
                    mouth_l_h_distance = self.safe_smooth(self.safe_average(self.mouth_l_h_avg), self.mouth_l_h_avg)
                    mouth_r_h_distance = self.safe_smooth(self.safe_average(self.mouth_r_h_avg), self.mouth_r_h_avg)
                    mouth_l_v_distance = self.safe_smooth(self.safe_average(self.mouth_l_v_avg), self.mouth_l_v_avg)
                    mouth_v_distance = self.safe_smooth(self.safe_average(self.mouth_v_avg), self.mouth_v_avg)
                    mouth_r_v_distance = self.safe_smooth(self.safe_average(self.mouth_r_v_avg), self.mouth_r_v_avg)
                    
                    # === EYES PROCESSING ===
                    left_iris = face_landmarks.landmark[468]
                    right_iris = face_landmarks.landmark[473]
                    left_eye_inner = face_landmarks.landmark[133]
                    left_eye_outer = face_landmarks.landmark[33]
                    right_eye_inner = face_landmarks.landmark[362]
                    right_eye_outer = face_landmarks.landmark[263]
                    left_upper_eyelid = face_landmarks.landmark[159]
                    right_upper_eyelid = face_landmarks.landmark[386]
                    left_lower_eyelid = face_landmarks.landmark[145]
                    right_lower_eyelid = face_landmarks.landmark[374]
                    
                    left_eyelid_distance = abs(left_upper_eyelid.y - left_lower_eyelid.y) / self.face_height
                    right_eyelid_distance = abs(right_upper_eyelid.y - right_lower_eyelid.y) / self.face_height
                    
                    left_eye_open = left_eyelid_distance > 0.03
                    right_eye_open = right_eyelid_distance > 0.03
                    
                    left_v = calculate_distance(left_upper_eyelid, left_lower_eyelid) / 2
                    right_v = calculate_distance(right_upper_eyelid, right_lower_eyelid) / 2
                    left_v_mid = (left_eye_outer.y + left_eye_inner.y) / 2
                    right_v_mid = (right_eye_outer.y + right_eye_inner.y) / 2
                    
                    left_horizontal = map_value(left_iris.x, left_eye_outer.x, left_eye_inner.x, 0, 100)
                    left_vertical = map_value(left_iris.y, left_v_mid - left_v, left_v_mid + left_v, 0, 100)
                    right_horizontal = map_value(right_iris.x, right_eye_inner.x, right_eye_outer.x, 0, 100)
                    right_vertical = map_value(right_iris.y, right_v_mid - right_v, right_v_mid + right_v, 0, 100)
                    
                    self.eyes_avg.append((left_horizontal, left_vertical, right_horizontal, right_vertical))
                    smoothed_eye_values = [sum(x[i] for x in self.eyes_avg) / len(self.eyes_avg) for i in range(4)]
                    position_lid = [1.0 if left_eye_open else 0.0, 1.0 if right_eye_open else 0.0]
                    
                    # === EYEBROW PROCESSING ===
                    brow_positions = self.calculate_eyebrow_positions(face_landmarks)
                    self.eyes_brow_avg.append(tuple(brow_positions))
                    smoothed_brow_values = [sum(x[i] for x in self.eyes_brow_avg) / len(self.eyes_brow_avg) for i in range(4)]
                    
                    # === PUBLISH VALUES ===
                    self.publish_eyes(smoothed_eye_values, position_lid)
                    self.publish_eyebrows(smoothed_brow_values)
                    self.publish_mouth(mouth_v_distance, mouth_l_h_distance, mouth_r_h_distance, mouth_l_v_distance, mouth_r_v_distance)
                    self.publish_jaw(mouth_v_distance)
                    
                except Exception as e:
                    self.get_logger().error(f"Error processing face landmarks: {e}")
        
        if self.show_video:
            cv2.imshow('Face Mimic', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cleanup()
    
    def publish_eyes(self, eye_pos, lid_pos):
        """Publish eye motor positions."""
        # Eyelid mapping
        # left_lid: 2268=CLOSE, 2058=OPEN (max=CLOSE)
        # right_lid: 938=CLOSE, 1172=OPEN (min=CLOSE)
        left_lid = map_value(lid_pos[0], 0, 1.1, rad(2268), rad(2058))  # 0=CLOSE, 1=OPEN
        right_lid = map_value(lid_pos[1], 0, 1.1, rad(938), rad(1172))  # 0=CLOSE, 1=OPEN
        
        # Eye position mapping
        # left_v: max=ABOVE (2300), min=BELOW (2000)
        # right_v: max=BELOW (370), min=ABOVE (59)
        # left_h: max=RIGHT (2000), min=LEFT (1436)
        # right_h: max=RIGHT (2459), min=LEFT (1950)
        left_h = map_value(eye_pos[0], 25, 65, rad(1436), rad(2000))   # low=LEFT, high=RIGHT
        left_v = map_value(eye_pos[1], 25, 65, rad(2000), rad(2300))   # low=BELOW, high=ABOVE
        right_h = map_value(eye_pos[0], 25, 65, rad(1950), rad(2459))  # low=LEFT, high=RIGHT
        right_v = map_value(eye_pos[1], 25, 65, rad(59), rad(370))     # low=ABOVE, high=BELOW
        
        msg = JointTrajectory()
        msg.joint_names = ["left_lid", "right_lid", "left_h", "left_v", "right_h", "right_v"]
        
        point = JointTrajectoryPoint()
        point.positions = [left_lid, right_lid, left_h, left_v, right_h, right_v]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        
        msg.points.append(point)
        self.publisher_eye.publish(msg)
    
    def publish_eyebrows(self, position):
        """Publish eyebrow motor positions.
        
        Motor direction mapping:
        - left_brow_1 (ID 7):  max=DOWN, min=UP (2414=DOWN, 1040=UP)
        - left_brow_2 (ID 8):  max=DOWN, min=UP (1777=DOWN, 1412=UP)
        - right_brow_1 (ID 9): min=DOWN, max=UP (1616=DOWN, 1907=UP)
        - right_brow_2 (ID 10): min=DOWN, max=UP (1440=DOWN, 1734=UP)
        """
        # Left brows: higher value = DOWN
        position[0] = map_value(position[0], 29, 33, rad(1040), rad(2414))  # Left inner: low=UP, high=DOWN
        position[1] = map_value(position[1], 24, 28, rad(1412), rad(1777))  # Left outer: low=UP, high=DOWN
        # Right brows: lower value = DOWN
        position[2] = map_value(position[2], 29, 33, rad(1907), rad(1616))  # Right inner: low=UP, high=DOWN
        position[3] = map_value(position[3], 24, 28, rad(1734), rad(1440))  # Right outer: low=UP, high=DOWN
        
        msg = JointTrajectory()
        msg.joint_names = ["left_brow_1", "left_brow_2", "right_brow_1", "right_brow_2"]
        
        point = JointTrajectoryPoint()
        point.positions = [position[0], position[1], position[2], position[3]]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        
        msg.points.append(point)
        self.publisher_eye_brow.publish(msg)
    
    def publish_mouth(self, lip_v, lip_l_h, lip_r_h, lip_l_v, lip_r_v):
        """Publish mouth motor positions."""
        # Lip Upper Mappings
        # lip_up_1: max=UP (2680), min=DOWN (1725)
        # lip_up_2: max=UP (2620), min=DOWN (1722)
        # lip_up_3: min=UP (1325), max=DOWN (2680)
        lip_up_1 = map_value(lip_l_v, 0.0, 0.15, 1725, 2680)    # low=DOWN, high=UP
        lip_up_2 = map_value(lip_v, 0.0, 0.10, 1722, 2620)      # low=DOWN, high=UP
        lip_up_3 = map_value(lip_r_v, 0.0, 0.15, 2680, 1325)    # low=DOWN, high=UP

        # Lip Lower Mappings
        # lip_down_1: max=DOWN (2209), min=UP (1497)
        # lip_down_2: min=DOWN (955), max=UP (1940)
        # lip_down_3: max=DOWN (2836), min=UP (2010)
        lip_down_1 = map_value(lip_l_v, 0.0, 0.15, 1497, 2209)  # low=UP, high=DOWN
        lip_down_2 = map_value(lip_v, 0.0, 0.10, 1940, 955)     # low=UP, high=DOWN
        lip_down_3 = map_value(lip_r_v, 0.0, 0.15, 2010, 2836)  # low=UP, high=DOWN
        
        # Cheek Mappings
        # left_cheek_up: min=UP (1555), max=DOWN (2230)
        # right_cheek_up: max=UP (2827), min=DOWN (2037)
        # left_cheek_down: min=UP (1060), max=DOWN (2413)
        # right_cheek_down: min=DOWN (1923), max=UP (3045)
        left_cheek_down = map_value(lip_l_h, 0.4, 0.65, 1060, 2413) # low=UP, high=DOWN
        left_cheek_up = map_value(lip_l_h, 0.4, 0.65, 1555, 2230)   # low=UP, high=DOWN
        right_cheek_down = map_value(lip_r_h, 0.4, 0.65, 1923, 3045)# low=DOWN, high=UP
        right_cheek_up = map_value(lip_r_h, 0.4, 0.65, 2827, 2037)  # low=UP, high=DOWN
        
        msg = JointTrajectory()
        msg.joint_names = [
            "lip_up_1", "lip_up_2", "lip_up_3",
            "lip_down_1", "lip_down_2", "lip_down_3",
            "left_cheek_down", "left_cheek_up",
            "right_cheek_down", "right_cheek_up"
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [
            rad(lip_up_1), rad(lip_up_2), rad(lip_up_3),
            rad(lip_down_1), rad(lip_down_2), rad(lip_down_3),
            rad(left_cheek_down), rad(left_cheek_up),
            rad(right_cheek_down), rad(right_cheek_up)
        ]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        
        msg.points.append(point)
        self.publisher_mouth.publish(msg)
    
    def publish_jaw(self, lip_v):
        """Publish jaw motor position."""
        jaw_value = map_value(lip_v, 0.0, 0.12, 2250, 2038)
        
        msg = JointTrajectory()
        msg.joint_names = ["jaw"]
        
        point = JointTrajectoryPoint()
        point.positions = [rad(jaw_value)]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        
        msg.points.append(point)
        self.publisher_jaw.publish(msg)
    
    def cleanup(self):
        """Clean up resources."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
    
    def destroy_node(self):
        """Cleanup on node destruction."""
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceMimicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
