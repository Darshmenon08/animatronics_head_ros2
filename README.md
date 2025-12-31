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

### Face Mimic (Landmark-based motor control)
```bash
# Default camera (0)
ros2 launch animatronics_head_ros2 face_mimic.launch.py

# With HP webcam (camera 9)
ros2 launch animatronics_head_ros2 face_mimic.launch.py camera_id:=9

# Without video preview
ros2 launch animatronics_head_ros2 face_mimic.launch.py show_video:=false
```

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

## License

TODO
