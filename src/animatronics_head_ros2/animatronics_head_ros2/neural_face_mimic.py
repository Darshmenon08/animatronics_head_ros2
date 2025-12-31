#!/usr/bin/env python3
"""
Neural Face Mimic Node for Animatronics Head
Uses a trained neural network to map face features to motor positions.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from builtin_interfaces.msg import Duration
import tensorflow as tf
import joblib
import numpy as np
import os

class NeuralFaceMimicNode(Node):
    def __init__(self):
        super().__init__('neural_face_mimic')
        
        # Parameters
        self.declare_parameter('model_path', 'models/face_model.h5')
        self.declare_parameter('scaler_x_path', 'models/scaler_X.pkl')
        self.declare_parameter('scaler_y_path', 'models/scaler_y.pkl')
        
        model_rel_path = self.get_parameter('model_path').value
        scaler_x_rel_path = self.get_parameter('scaler_x_path').value
        scaler_y_rel_path = self.get_parameter('scaler_y_path').value
        
        base_path = os.path.expanduser('~/animatronics_head_ros2')
        self.model_path = os.path.join(base_path, model_rel_path)
        self.scaler_x_path = os.path.join(base_path, scaler_x_rel_path)
        self.scaler_y_path = os.path.join(base_path, scaler_y_rel_path)
        
        # Load model and scalers
        try:
            self.model = tf.keras.models.load_model(self.model_path)
            self.scaler_x = joblib.load(self.scaler_x_path)
            self.scaler_y = joblib.load(self.scaler_y_path)
            self.get_logger().info("Model and scalers loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model or scalers: {e}")
            raise
            
        # Publishers
        self.publisher_eye = self.create_publisher(JointTrajectory, '/eye/joint_trajectory', 10)
        self.publisher_jaw = self.create_publisher(JointTrajectory, '/jaw/joint_trajectory', 10)
        self.publisher_eye_brow = self.create_publisher(JointTrajectory, '/nose_eye_brow/joint_trajectory', 10)
        self.publisher_mouth = self.create_publisher(JointTrajectory, '/mouth/joint_trajectory', 10)
        
        # Subscriber
        self.create_subscription(Float32MultiArray, '/face_features', self.features_callback, 10)
        
        self.motor_names = [
            "left_lid", "right_lid", "left_h", "left_v", "right_h", "right_v",
            "left_brow_1", "left_brow_2", "right_brow_1", "right_brow_2",
            "lip_up_1", "lip_up_2", "lip_up_3",
            "lip_down_1", "lip_down_2", "lip_down_3",
            "left_cheek_down", "left_cheek_up",
            "right_cheek_down", "right_cheek_up",
            "jaw"
        ]

    def features_callback(self, msg):
        if len(msg.data) != 15:
            return
            
        # Prepare input
        features = np.array(msg.data).reshape(1, -1)
        
        # Normalize input
        features_scaled = self.scaler_x.transform(features)
        
        # Predict
        prediction_scaled = self.model.predict(features_scaled, verbose=0)
        
        # Denormalize output
        prediction = self.scaler_y.inverse_transform(prediction_scaled)[0]
        
        # Map prediction to motors
        motors = dict(zip(self.motor_names, prediction))
        
        self.publish_motors(motors)

    def publish_motors(self, motors):
        # Eyes
        msg_eye = JointTrajectory()
        msg_eye.joint_names = ["left_lid", "right_lid", "left_h", "left_v", "right_h", "right_v"]
        point_eye = JointTrajectoryPoint()
        point_eye.positions = [
            motors["left_lid"], motors["right_lid"],
            motors["left_h"], motors["left_v"],
            motors["right_h"], motors["right_v"]
        ]
        point_eye.time_from_start = Duration(sec=0, nanosec=100000000)
        msg_eye.points.append(point_eye)
        self.publisher_eye.publish(msg_eye)
        
        # Brows
        msg_brow = JointTrajectory()
        msg_brow.joint_names = ["left_brow_1", "left_brow_2", "right_brow_1", "right_brow_2"]
        point_brow = JointTrajectoryPoint()
        point_brow.positions = [
            motors["left_brow_1"], motors["left_brow_2"],
            motors["right_brow_1"], motors["right_brow_2"]
        ]
        point_brow.time_from_start = Duration(sec=0, nanosec=100000000)
        msg_brow.points.append(point_brow)
        self.publisher_eye_brow.publish(msg_brow)
        
        # Mouth
        msg_mouth = JointTrajectory()
        msg_mouth.joint_names = [
            "lip_up_1", "lip_up_2", "lip_up_3",
            "lip_down_1", "lip_down_2", "lip_down_3",
            "left_cheek_down", "left_cheek_up",
            "right_cheek_down", "right_cheek_up"
        ]
        point_mouth = JointTrajectoryPoint()
        point_mouth.positions = [
            motors["lip_up_1"], motors["lip_up_2"], motors["lip_up_3"],
            motors["lip_down_1"], motors["lip_down_2"], motors["lip_down_3"],
            motors["left_cheek_down"], motors["left_cheek_up"],
            motors["right_cheek_down"], motors["right_cheek_up"]
        ]
        point_mouth.time_from_start = Duration(sec=0, nanosec=100000000)
        msg_mouth.points.append(point_mouth)
        self.publisher_mouth.publish(msg_mouth)
        
        # Jaw
        msg_jaw = JointTrajectory()
        msg_jaw.joint_names = ["jaw"]
        point_jaw = JointTrajectoryPoint()
        point_jaw.positions = [motors["jaw"]]
        point_jaw.time_from_start = Duration(sec=0, nanosec=100000000)
        msg_jaw.points.append(point_jaw)
        self.publisher_jaw.publish(msg_jaw)

def main(args=None):
    rclpy.init(args=args)
    node = NeuralFaceMimicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
