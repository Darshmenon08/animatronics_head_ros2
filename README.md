# Animatronics Head ROS2

ROS2 package for controlling an animatronics head with 25 Dynamixel motors.

## Features

- **Motor Slider Control**: GUI with sliders to control all 25 motors
- **Face Mimic**: Real-time face tracking using MediaPipe to control motors
- **Blendshape Mimic**: Uses trained neural network to map blendshapes to motors
- **Motor Value Checker**: Utility to read current motor positions
- **Dynamixel Controller**: Low-level SDK controller for motor communication

## Motor Configuration

| Port | Motors | IDs |
|------|--------|-----|
| `/dev/eye` | Eyes (h, v, lids) | 1-6 |
| `/dev/nose_eye_brow` | Eyebrows, Nose | 7-12 |
| `/dev/mouth` | Lips, Cheeks, Tongue | 13-24 |
| `/dev/jaw` | Jaw | 25 |

## Installation

```bash
# Install dependencies
pip install mediapipe dynamixel-sdk

# Build the package
cd ~/animatronics_head_ros2
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Hardware Interface (Required for Face Mimic)
This launches the motor controller and hardware communication nodes.
```bash
ros2 launch animatronics_head_ros2 bringup.launch.py
```

### Motor Slider Control (Standalone)
**Note**: Do not run this at the same time as the Hardware Interface.
```bash
ros2 launch animatronics_head_ros2 slider_control.launch.py
```

See [docs/MOTOR_CALIBRATION.md](docs/MOTOR_CALIBRATION.md) for detailed motor calibration and direction information.

### Face Mimic (Complete System)
This launches BOTH the hardware interface and the face tracking logic.

```bash
# Default camera (0)
ros2 launch animatronics_head_ros2 mimic_complete.launch.py

# With HP 320 FHD Webcam (camera 9, symlinked to /dev/webcam via udev)
ros2 launch animatronics_head_ros2 mimic_complete.launch.py camera_id:=9

# Without video preview
ros2 launch animatronics_head_ros2 mimic_complete.launch.py show_video:=false
```

> **Note**: The `camera_id` is the V4L2 device index. To find your webcam's index, run:  
> `ls -la /dev/video*` or check with `v4l2-ctl --list-devices`

### Motor Value Checker
```bash
ros2 run animatronics_head_ros2 motor_value_checker
```

### MediaPipe Test (Landmarks + Blendshapes)
```bash
python3 src/animatronics_head_ros2/animatronics_head_ros2/mediapipe_test.py
```
Press 'q' to quit.

## Udev Rules

Copy the udev rules to enable motor and webcam communication:
```bash
# Motor udev rules
sudo cp src/animatronics_head_ros2/config/head.rules /etc/udev/rules.d/99-head.rules

# Webcam udev rule (creates /dev/webcam symlink)
sudo cp src/animatronics_head_ros2/config/webcam.rules /etc/udev/rules.d/99-webcam.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Durability Testing

Test motor durability by recording your expressions and playing them back on loop.

### 1. Record Expressions
```bash
python3 src/animatronics_head_ros2/animatronics_head_ros2/record_expression.py
```
- Press **'r'** to start/stop recording
- Press **'q'** to quit
- Saves to `~/animatronics_head_ros2/recordings/`

### 2. Playback on Loop
```bash
# Uses latest recording, loops infinitely
ros2 launch animatronics_head_ros2 video_mimic.launch.py

# With specific video
ros2 launch animatronics_head_ros2 video_mimic.launch.py video_path:=/path/to/video.mp4

# Headless (no video preview)
ros2 launch animatronics_head_ros2 video_mimic.launch.py show_video:=false
```
Press **Ctrl+C** to stop. Use for 30+ minute durability tests.

## License


## Neural Network Training

You can train a neural network to learn the mapping between your face and the robot's motors.



### 1. Data Collection
To collect training data, run this single command. It will launch the camera, the sliders, and the recorder.
```bash
ros2 launch animatronics_head_ros2 collect_data.launch.py
```
**Action**: Move your face in front of the camera while simultaneously adjusting the sliders to match the robot's expression to yours. The data collector will save the correspondence to a CSV file in `~/animatronics_head_ros2/data/`.



### 2. Training
Once you have collected enough data (multiple CSV files are fine), run the training script:
```bash
python3 src/animatronics_head_ros2/animatronics_head_ros2/train_model.py
```
This will train a model and save it to `~/animatronics_head_ros2/models/face_model.h5`.

### 3. Inference (Neural Mimic)
To control the robot using the trained neural network:
```bash
ros2 run animatronics_head_ros2 neural_face_mimic
```
Make sure `face_mimic` is also running (it provides the input features):
```bash
ros2 run animatronics_head_ros2 face_mimic --ros-args -p drive_motors:=False
```

## License

TODO
