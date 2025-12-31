#!/usr/bin/env python3
"""
MediaPipe Face Test - Displays landmarks AND blendshapes from webcam
Uses MediaPipe Tasks API for blendshape support
"""

import cv2
import mediapipe as mp
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import urllib.request
import os


# Blendshape names for display
BLENDSHAPE_NAMES = [
    "browDownLeft", "browDownRight", "browInnerUp", "browOuterUpLeft", "browOuterUpRight",
    "cheekPuff", "cheekSquintLeft", "cheekSquintRight", "eyeBlinkLeft", "eyeBlinkRight",
    "eyeLookDownLeft", "eyeLookDownRight", "eyeLookInLeft", "eyeLookInRight",
    "eyeLookOutLeft", "eyeLookOutRight", "eyeLookUpLeft", "eyeLookUpRight",
    "eyeSquintLeft", "eyeSquintRight", "eyeWideLeft", "eyeWideRight",
    "jawForward", "jawLeft", "jawOpen", "jawRight",
    "mouthClose", "mouthDimpleLeft", "mouthDimpleRight", "mouthFrownLeft", "mouthFrownRight",
    "mouthFunnel", "mouthLeft", "mouthLowerDownLeft", "mouthLowerDownRight",
    "mouthPressLeft", "mouthPressRight", "mouthPucker", "mouthRight",
    "mouthRollLower", "mouthRollUpper", "mouthShrugLower", "mouthShrugUpper",
    "mouthSmileLeft", "mouthSmileRight", "mouthStretchLeft", "mouthStretchRight",
    "mouthUpperUpLeft", "mouthUpperUpRight", "noseSneerLeft", "noseSneerRight"
]


def download_model():
    """Download the face landmarker model if not present."""
    model_path = "/tmp/face_landmarker_v2_with_blendshapes.task"
    if not os.path.exists(model_path):
        print("Downloading face landmarker model with blendshapes...")
        url = "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task"
        urllib.request.urlretrieve(url, model_path)
        print("Model downloaded!")
    return model_path


def draw_landmarks_on_image(rgb_image, detection_result):
    """Draw face landmarks on image."""
    face_landmarks_list = detection_result.face_landmarks
    annotated_image = np.copy(rgb_image)

    for face_landmarks in face_landmarks_list:
        face_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        face_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) 
            for landmark in face_landmarks
        ])

        solutions.drawing_utils.draw_landmarks(
            image=annotated_image,
            landmark_list=face_landmarks_proto,
            connections=mp.solutions.face_mesh.FACEMESH_TESSELATION,
            landmark_drawing_spec=None,
            connection_drawing_spec=mp.solutions.drawing_styles.get_default_face_mesh_tesselation_style()
        )
        solutions.drawing_utils.draw_landmarks(
            image=annotated_image,
            landmark_list=face_landmarks_proto,
            connections=mp.solutions.face_mesh.FACEMESH_CONTOURS,
            landmark_drawing_spec=None,
            connection_drawing_spec=mp.solutions.drawing_styles.get_default_face_mesh_contours_style()
        )
        solutions.drawing_utils.draw_landmarks(
            image=annotated_image,
            landmark_list=face_landmarks_proto,
            connections=mp.solutions.face_mesh.FACEMESH_IRISES,
            landmark_drawing_spec=None,
            connection_drawing_spec=mp.solutions.drawing_styles.get_default_face_mesh_iris_connections_style()
        )

    return annotated_image


def main():
    camera_id = 9  # HP 320 FHD Webcam (/dev/webcam)
    
    # Download model
    model_path = download_model()
    
    # Create FaceLandmarker with blendshapes
    from mediapipe.tasks import python
    from mediapipe.tasks.python import vision
    
    base_options = python.BaseOptions(model_asset_path=model_path)
    options = vision.FaceLandmarkerOptions(
        base_options=base_options,
        output_face_blendshapes=True,
        output_facial_transformation_matrixes=True,
        num_faces=1
    )
    detector = vision.FaceLandmarker.create_from_options(options)
    
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Failed to open camera {camera_id}, trying 0...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("ERROR: Could not open any camera!")
            return
    
    print("Camera opened successfully!")
    print("Press 'q' to quit")
    print("Showing: LANDMARKS + BLENDSHAPES")
    
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            continue
        
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        
        # Detect face landmarks and blendshapes
        detection_result = detector.detect(mp_image)
        
        if detection_result.face_landmarks:
            # Draw landmarks
            annotated_frame = draw_landmarks_on_image(rgb_frame, detection_result)
            annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
            
            # Display landmark count
            num_landmarks = len(detection_result.face_landmarks[0])
            cv2.putText(annotated_frame, f"Landmarks: {num_landmarks}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display blendshapes
            if detection_result.face_blendshapes:
                blendshapes = detection_result.face_blendshapes[0]
                
                # Sort by score and show top 12
                sorted_bs = sorted(blendshapes, key=lambda x: x.score, reverse=True)[:12]
                
                y_offset = 60
                cv2.putText(annotated_frame, "Top Blendshapes:", (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                y_offset += 25
                
                for bs in sorted_bs:
                    # Draw bar graph
                    bar_width = int(bs.score * 150)
                    cv2.rectangle(annotated_frame, (10, y_offset - 12), (10 + bar_width, y_offset), (0, 255, 255), -1)
                    text = f"{bs.category_name}: {bs.score:.2f}"
                    cv2.putText(annotated_frame, text, (170, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    y_offset += 18
        else:
            annotated_frame = frame
            cv2.putText(annotated_frame, "No face detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow('MediaPipe - Landmarks & Blendshapes', annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    detector.close()
    print("Done!")


if __name__ == '__main__':
    main()
