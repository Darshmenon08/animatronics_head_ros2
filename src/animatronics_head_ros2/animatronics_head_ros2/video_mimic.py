#!/usr/bin/env python3
"""
Video Mimic Node
Plays back a recorded video and mimics expressions on the animatronic head.
Use this for testing durability and motion quality with repeatable expressions.
Can loop the video for long-duration testing.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import cv2
import os
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


def calculate_distance(point1, point2):
    """Euclidean distance between two points."""
    return (((point1.x - point2.x) ** 2) + ((point1.y - point2.y) ** 2)) ** 0.5


def map_value(value, in_min, in_max, out_min, out_max):
    """Map value from one range to another, clamped to input range."""
    value = max(min(value, in_max), in_min)
    return round((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, 5)


class VideoMimicNode(Node):
    def __init__(self):
        super().__init__('video_mimic')
        
        # Declare parameters
        self.declare_parameter('video_path', '')
        self.declare_parameter('loop', True)
        self.declare_parameter('loop_count', 0)  # 0 = infinite
        self.declare_parameter('show_video', True)
        self.declare_parameter('playback_speed', 1.0)
        
        self.video_path = self.get_parameter('video_path').value
        self.loop = self.get_parameter('loop').value
        self.loop_count = self.get_parameter('loop_count').value
        self.show_video = self.get_parameter('show_video').value
        self.playback_speed = self.get_parameter('playback_speed').value
        
        if not self.video_path:
            # Try to find latest recording
            recordings_dir = os.path.expanduser("~/animatronics_head_ros2/recordings")
            if os.path.exists(recordings_dir):
                videos = sorted([f for f in os.listdir(recordings_dir) if f.endswith('.mp4')])
                if videos:
                    self.video_path = os.path.join(recordings_dir, videos[-1])
                    self.get_logger().info(f"Using latest recording: {self.video_path}")
        
        if not self.video_path or not os.path.exists(self.video_path):
            self.get_logger().error(f"Video file not found: {self.video_path}")
            raise FileNotFoundError(f"Video not found: {self.video_path}")
        
        # Publishers
        self.publisher_eye = self.create_publisher(JointTrajectory, '/eye/joint_trajectory', 10)
        self.publisher_jaw = self.create_publisher(JointTrajectory, '/jaw/joint_trajectory', 10)
        self.publisher_eye_brow = self.create_publisher(JointTrajectory, '/nose_eye_brow/joint_trajectory', 10)
        self.publisher_mouth = self.create_publisher(JointTrajectory, '/mouth/joint_trajectory', 10)
        
        # MediaPipe setup
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Video capture
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open video: {self.video_path}")
        
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.current_loop = 0
        
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.face_width = 1.0
        self.face_height = 1.0
        
        # Smoothing buffers
        self.q_size = 10
        self.eyes_avg = deque(maxlen=self.q_size)
        self.eyes_brow_avg = deque(maxlen=self.q_size)
        
        # Timer for processing frames
        timer_period = (1.0 / self.fps) / self.playback_speed
        self.create_timer(timer_period, self.process_frame)
        
        self.get_logger().info(f'Video Mimic started: {self.video_path}')
        self.get_logger().info(f'Total frames: {self.total_frames}, FPS: {self.fps}')
        self.get_logger().info(f'Loop: {self.loop}, Loop count: {self.loop_count} (0=infinite)')
    
    def process_frame(self):
        """Process a single frame from the video."""
        ret, frame = self.cap.read()
        
        if not ret:
            if self.loop:
                if self.loop_count == 0 or self.current_loop < self.loop_count - 1:
                    self.current_loop += 1
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self.get_logger().info(f"Loop {self.current_loop} started")
                    return
            self.get_logger().info("Video playback complete")
            rclpy.shutdown()
            return
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(frame_rgb)
        
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                try:
                    self.face_width = abs(face_landmarks.landmark[361].x - face_landmarks.landmark[133].x)
                    self.face_height = abs(face_landmarks.landmark[10].y - face_landmarks.landmark[152].y)
                    
                    # Process and publish all features (same as face_mimic.py)
                    self.process_eyes(face_landmarks)
                    self.process_eyebrows(face_landmarks)
                    self.process_mouth(face_landmarks)
                    
                except Exception as e:
                    self.get_logger().error(f"Error processing: {e}")
        
        if self.show_video:
            # Add overlay info
            frame_num = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
            info = f"Frame: {frame_num}/{self.total_frames} | Loop: {self.current_loop}"
            cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Video Mimic', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
    
    def process_eyes(self, face_landmarks):
        """Process eye landmarks and publish."""
        # Get landmarks
        left_iris = face_landmarks.landmark[468]
        right_iris = face_landmarks.landmark[473]
        left_eye_inner = face_landmarks.landmark[133]
        left_eye_outer = face_landmarks.landmark[33]
        right_eye_inner = face_landmarks.landmark[362]
        right_eye_outer = face_landmarks.landmark[263]
        left_upper_eyelid = face_landmarks.landmark[159]
        left_lower_eyelid = face_landmarks.landmark[145]
        right_upper_eyelid = face_landmarks.landmark[386]
        right_lower_eyelid = face_landmarks.landmark[374]
        
        # Eyelid openness
        left_eyelid_distance = abs(left_upper_eyelid.y - left_lower_eyelid.y) / self.face_height
        right_eyelid_distance = abs(right_upper_eyelid.y - right_lower_eyelid.y) / self.face_height
        left_lid_val = map_value(left_eyelid_distance, 0.015, 0.07, 0.0, 1.0)
        right_lid_val = map_value(right_eyelid_distance, 0.015, 0.07, 0.0, 1.0)
        
        # Eye position
        left_v = calculate_distance(left_upper_eyelid, left_lower_eyelid) / 2
        right_v = calculate_distance(right_upper_eyelid, right_lower_eyelid) / 2
        left_v_mid = (left_eye_outer.y + left_eye_inner.y) / 2
        right_v_mid = (right_eye_outer.y + right_eye_inner.y) / 2
        
        left_horizontal = map_value(left_iris.x, left_eye_outer.x, left_eye_inner.x, 0, 100)
        left_vertical = map_value(left_iris.y, left_v_mid - left_v, left_v_mid + left_v, 0, 100)
        right_horizontal = map_value(right_iris.x, right_eye_inner.x, right_eye_outer.x, 0, 100)
        right_vertical = map_value(right_iris.y, right_v_mid - right_v, right_v_mid + right_v, 0, 100)
        
        self.eyes_avg.append((left_horizontal, left_vertical, right_horizontal, right_vertical))
        smoothed = [sum(x[i] for x in self.eyes_avg) / len(self.eyes_avg) for i in range(4)]
        
        # Publish
        left_lid = map_value(left_lid_val, 0.0, 1.0, rad(2268), rad(2058))
        right_lid = map_value(right_lid_val, 0.0, 1.0, rad(938), rad(1172))
        left_h = map_value(smoothed[0], 25, 65, rad(1436), rad(2000))
        left_v = map_value(smoothed[1], 25, 65, rad(2000), rad(2300))
        right_h = map_value(smoothed[0], 25, 65, rad(1950), rad(2459))
        right_v = map_value(smoothed[1], 25, 65, rad(59), rad(370))
        
        msg = JointTrajectory()
        msg.joint_names = ["left_lid", "right_lid", "left_h", "left_v", "right_h", "right_v"]
        point = JointTrajectoryPoint()
        point.positions = [left_lid, right_lid, left_h, left_v, right_h, right_v]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        msg.points.append(point)
        self.publisher_eye.publish(msg)
    
    def process_eyebrows(self, face_landmarks):
        """Process eyebrow landmarks and publish."""
        left_brow_landmarks = [
            face_landmarks.landmark[107],
            face_landmarks.landmark[66],
            face_landmarks.landmark[105],
            face_landmarks.landmark[63],
            face_landmarks.landmark[70]
        ]
        right_brow_landmarks = [
            face_landmarks.landmark[336],
            face_landmarks.landmark[296],
            face_landmarks.landmark[334],
            face_landmarks.landmark[293],
            face_landmarks.landmark[300]
        ]
        forehead_center = face_landmarks.landmark[10]
        
        left_brow_heights = [(l.y - forehead_center.y) / self.face_height for l in left_brow_landmarks]
        right_brow_heights = [(l.y - forehead_center.y) / self.face_height for l in right_brow_landmarks]
        
        positions = [
            max(20, min(80, (left_brow_heights[0] + 0.1) * 100)),
            max(20, min(80, (left_brow_heights[-1] + 0.1) * 100)),
            max(20, min(80, (right_brow_heights[0] + 0.1) * 100)),
            max(20, min(80, (right_brow_heights[-1] + 0.1) * 100))
        ]
        
        self.eyes_brow_avg.append(tuple(positions))
        smoothed = [sum(x[i] for x in self.eyes_brow_avg) / len(self.eyes_brow_avg) for i in range(4)]
        
        msg = JointTrajectory()
        msg.joint_names = ["left_brow_1", "left_brow_2", "right_brow_1", "right_brow_2"]
        point = JointTrajectoryPoint()
        point.positions = [
            map_value(smoothed[0], 29, 33, rad(2040), rad(2414)),
            map_value(smoothed[1], 24, 28, rad(1412), rad(1777)),
            map_value(smoothed[2], 29, 33, rad(1907), rad(1616)),
            map_value(smoothed[3], 24, 28, rad(1734), rad(1440))
        ]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        msg.points.append(point)
        self.publisher_eye_brow.publish(msg)
    
    def process_mouth(self, face_landmarks):
        """Process mouth landmarks and publish."""
        upper_lip = face_landmarks.landmark[13]
        lower_lip = face_landmarks.landmark[14]
        left_lip = face_landmarks.landmark[308]
        right_lip = face_landmarks.landmark[62]
        
        class MidLip:
            pass
        mid_lip = MidLip()
        mid_lip.x = (left_lip.x + right_lip.x) / 2
        mid_lip.y = (left_lip.y + right_lip.y) / 2
        
        mouth_v = calculate_distance(upper_lip, lower_lip) / self.face_height
        mouth_l_h = calculate_distance(left_lip, mid_lip) * 2 / self.face_width
        mouth_r_h = calculate_distance(mid_lip, right_lip) * 2 / self.face_width
        
        # Lip mappings
        lip_up_1 = map_value(mouth_v, 0.0, 0.15, 1725, 2680)
        lip_up_2 = map_value(mouth_v, 0.0, 0.10, 1722, 2620)
        lip_up_3 = map_value(mouth_v, 0.0, 0.15, 2680, 1325)
        lip_down_1 = map_value(mouth_v, 0.0, 0.15, 1497, 2209)
        lip_down_2 = map_value(mouth_v, 0.0, 0.10, 1940, 955)
        lip_down_3 = map_value(mouth_v, 0.0, 0.15, 2010, 2836)
        
        # Cheek mappings
        left_cheek_down = map_value(mouth_l_h, 0.4, 0.65, 2413, 1060)
        left_cheek_up = map_value(mouth_l_h, 0.4, 0.65, 2230, 1555)
        right_cheek_down = map_value(mouth_r_h, 0.4, 0.65, 1923, 3045)
        right_cheek_up = map_value(mouth_r_h, 0.4, 0.65, 2037, 2827)
        
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
        
        # Jaw
        jaw_value = map_value(mouth_v, 0.0, 0.12, 2300, 2000)
        jaw_msg = JointTrajectory()
        jaw_msg.joint_names = ["jaw"]
        jaw_point = JointTrajectoryPoint()
        jaw_point.positions = [rad(jaw_value)]
        jaw_point.time_from_start = Duration(sec=0, nanosec=100000000)
        jaw_msg.points.append(jaw_point)
        self.publisher_jaw.publish(jaw_msg)
    
    def destroy_node(self):
        """Cleanup on node destruction."""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoMimicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
