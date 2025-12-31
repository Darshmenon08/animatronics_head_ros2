#!/usr/bin/env python3
"""
Record Expression Script
Records webcam video for later playback testing of the animatronic head.
Press 'r' to start/stop recording, 'q' to quit.
"""

import cv2
import os
from datetime import datetime


def main():
    camera_id = 9  # HP 320 FHD Webcam
    output_dir = os.path.expanduser("~/animatronics_head_ros2/recordings")
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Failed to open camera {camera_id}, trying 0...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Failed to open any camera!")
            return
    
    # Get camera properties
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = 30
    
    recording = False
    out = None
    filename = None
    
    print("=" * 50)
    print("Expression Recorder")
    print("=" * 50)
    print("Controls:")
    print("  'r' - Start/Stop recording")
    print("  'q' - Quit")
    print("=" * 50)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Add status overlay
        status = "RECORDING" if recording else "READY"
        color = (0, 0, 255) if recording else (0, 255, 0)
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        if recording:
            # Add recording indicator (red dot)
            cv2.circle(frame, (frame_width - 30, 30), 15, (0, 0, 255), -1)
            out.write(frame)
        
        cv2.imshow('Expression Recorder', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('r'):
            if not recording:
                # Start recording
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(output_dir, f"expression_{timestamp}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                out = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))
                recording = True
                print(f"Started recording: {filename}")
            else:
                # Stop recording
                recording = False
                out.release()
                out = None
                print(f"Saved recording: {filename}")
        
        elif key == ord('q'):
            break
    
    # Cleanup
    if out is not None:
        out.release()
    cap.release()
    cv2.destroyAllWindows()
    
    print("\nRecordings saved to:", output_dir)


if __name__ == '__main__':
    main()
