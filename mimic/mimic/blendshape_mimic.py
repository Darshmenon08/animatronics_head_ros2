#!/usr/bin/env python3
"""
Blendshape Model Mimic Node for Animatronics Head (ROS2)
Uses a trained neural network to map blendshape values to motor positions.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import cv2
import math
import numpy as np

try:
    import mediapipe as mp
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
except ImportError:
    print("ERROR: mediapipe not found. Install with: pip install mediapipe")
    raise

try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("WARNING: PyTorch not found. Trying TensorFlow...")

try:
    import tensorflow as tf
    TF_AVAILABLE = True
except ImportError:
    TF_AVAILABLE = False


def rad(angle):
    """Convert a value in [0, 4095] to radians, centered at 0."""
    return (((angle * 360) / 4095) - 180) * math.pi / 180


class BlendshapeToMotorTorch(nn.Module):
    """PyTorch model for blendshape to motor mapping."""
    def __init__(self, input_size=52, output_size=12):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_size, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, output_size)
        )
    
    def forward(self, x):
        return self.fc(x)


class BlendshapeMimicNode(Node):
    def __init__(self):
        super().__init__('blendshape_mimic')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_type', 'tensorflow')  # 'tensorflow' or 'pytorch'
        self.declare_parameter('face_landmarker_path', '')
        self.declare_parameter('show_video', True)
        
        self.camera_id = self.get_parameter('camera_id').value
        self.model_path = self.get_parameter('model_path').value
        self.model_type = self.get_parameter('model_type').value
        self.face_landmarker_path = self.get_parameter('face_landmarker_path').value
        self.show_video = self.get_parameter('show_video').value
        
        # Publishers
        self.publisher_mouth = self.create_publisher(JointTrajectory, '/mouth/joint_trajectory', 10)
        self.publisher_jaw = self.create_publisher(JointTrajectory, '/jaw/joint_trajectory', 10)
        
        # Load the model
        self.model = None
        if self.model_path:
            self.load_model()
        else:
            self.get_logger().warn("No model path specified. Running without neural network.")
        
        # Initialize MediaPipe FaceLandmarker with blendshapes
        if self.face_landmarker_path:
            base_options = python.BaseOptions(model_asset_path=self.face_landmarker_path)
            options = vision.FaceLandmarkerOptions(
                base_options=base_options,
                output_face_blendshapes=True,
                num_faces=1
            )
            self.detector = vision.FaceLandmarker.create_from_options(options)
        else:
            self.get_logger().error("No face landmarker path specified!")
            self.detector = None
        
        # Video capture
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {self.camera_id}")
        
        # Motor joint names
        self.motor_names = [
            "jaw", "lip_up_1", "lip_up_2", "lip_up_3",
            "lip_down_1", "lip_down_2", "lip_down_3",
            "left_cheek_down", "left_cheek_up",
            "right_cheek_down", "right_cheek_up", "tongue_in_out"
        ]
        
        # Timer for processing frames (30 Hz)
        self.create_timer(1.0 / 30.0, self.process_frame)
        
        self.get_logger().info('Blendshape Mimic Node started')
    
    def load_model(self):
        """Load the neural network model."""
        try:
            if self.model_type == 'pytorch' and TORCH_AVAILABLE:
                self.model = BlendshapeToMotorTorch()
                self.model.load_state_dict(torch.load(self.model_path))
                self.model.eval()
                self.get_logger().info(f"Loaded PyTorch model from {self.model_path}")
            elif self.model_type == 'tensorflow' and TF_AVAILABLE:
                self.model = tf.keras.models.load_model(self.model_path)
                self.get_logger().info(f"Loaded TensorFlow model from {self.model_path}")
            else:
                self.get_logger().error(f"Cannot load model. Type: {self.model_type}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
    
    def get_blendshape_values(self, face_blendshapes):
        """Extract blendshape values from detection result."""
        return [blendshape.score for blendshape in face_blendshapes]
    
    def predict_motor_values(self, blendshape_values):
        """Predict motor values from blendshape values using the model."""
        if self.model is None:
            return None
        
        try:
            if self.model_type == 'pytorch' and TORCH_AVAILABLE:
                inputs = torch.tensor(blendshape_values, dtype=torch.float32).unsqueeze(0)
                with torch.no_grad():
                    outputs = self.model(inputs).squeeze(0).numpy()
            else:
                inputs = np.array(blendshape_values).reshape(1, -1)
                outputs = self.model.predict(inputs, verbose=0)[0]
            
            return outputs
        except Exception as e:
            self.get_logger().error(f"Prediction error: {e}")
            return None
    
    def process_frame(self):
        """Process a single frame from the camera."""
        if self.detector is None:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        
        # Detect face and blendshapes
        detection_result = self.detector.detect(image)
        
        if detection_result.face_blendshapes:
            blendshape_values = self.get_blendshape_values(detection_result.face_blendshapes[0])
            motor_values = self.predict_motor_values(blendshape_values)
            
            if motor_values is not None:
                self.publish_motor_values(motor_values)
        
        if self.show_video:
            cv2.imshow("Blendshape Mimic", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cleanup()
    
    def publish_motor_values(self, motor_values):
        """Publish motor values to ROS topics."""
        # Separate jaw from mouth motors
        jaw_msg = JointTrajectory()
        jaw_msg.joint_names = ["jaw"]
        jaw_point = JointTrajectoryPoint()
        jaw_point.positions = [rad(motor_values[0])]
        jaw_point.time_from_start = Duration(sec=0, nanosec=100000000)
        jaw_msg.points.append(jaw_point)
        self.publisher_jaw.publish(jaw_msg)
        
        # Mouth motors
        mouth_msg = JointTrajectory()
        mouth_msg.joint_names = self.motor_names[1:]  # Exclude jaw
        mouth_point = JointTrajectoryPoint()
        mouth_point.positions = [rad(v) for v in motor_values[1:]]
        mouth_point.time_from_start = Duration(sec=0, nanosec=100000000)
        mouth_msg.points.append(mouth_point)
        self.publisher_mouth.publish(mouth_msg)
    
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
    node = BlendshapeMimicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
