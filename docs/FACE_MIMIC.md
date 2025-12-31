# Face Mimic System Documentation

This document explains how the face mimic system works to translate human facial expressions into robot motor positions.

## Overview

The face mimic system uses **MediaPipe Face Mesh** to detect 478 facial landmarks from a webcam feed, calculates facial features (eye positions, mouth openness, eyebrow height, etc.), and maps them to the 25 Dynamixel motors.

## Architecture

```
┌─────────────┐    ┌──────────────┐    ┌────────────────┐    ┌─────────────┐
│   Camera    │───▶│  MediaPipe   │───▶│  face_mimic    │───▶│   Motors    │
│  (webcam)   │    │  Face Mesh   │    │    Node        │    │  (Dynamixel)│
└─────────────┘    └──────────────┘    └────────────────┘    └─────────────┘
                         │                     │
                         ▼                     ▼
                   478 Landmarks        JointTrajectory
                   + Blendshapes         Messages
```

## Feature Extraction

### Eyes (6 motors)
| Feature | Landmarks Used | Maps To |
|---------|---------------|---------|
| Horizontal gaze | Iris position relative to eye corners (468, 473, 33, 133, 263, 362) | left_h, right_h |
| Vertical gaze | Iris position relative to eye top/bottom (468, 473, 159, 145, 386, 374) | left_v, right_v |
| Eyelid openness | Distance between upper/lower eyelid (159-145, 386-374) | left_lid, right_lid |

**Code logic:**
```python
# Eyelid distance normalized by face height
left_eyelid_distance = abs(upper_eyelid.y - lower_eyelid.y) / face_height
# Typical range: 0.015 (closed) to 0.07 (open)
left_lid_val = map_value(left_eyelid_distance, 0.015, 0.07, 0.0, 1.0)
```

### Eyebrows (4 motors)
| Feature | Landmarks Used | Maps To |
|---------|---------------|---------|
| Left inner brow height | Landmark 107 relative to forehead (10) | left_brow_1 |
| Left outer brow height | Landmark 70 relative to forehead | left_brow_2 |
| Right inner brow height | Landmark 336 relative to forehead | right_brow_1 |
| Right outer brow height | Landmark 300 relative to forehead | right_brow_2 |

**Code logic:**
```python
# Eyebrow height as percentage of face
brow_height = (landmark.y - forehead_center.y) / face_height
# Output range: ~20-80 (normalized value)
```

### Mouth (10 motors)
| Feature | Landmarks Used | Maps To |
|---------|---------------|---------|
| Mouth vertical openness | Upper lip (13) to lower lip (14) | lip_up_2, lip_down_2, jaw |
| Left mouth corner | Left corner (308) to center | lip_up_1, lip_down_1, left_cheek_* |
| Right mouth corner | Right corner (62) to center | lip_up_3, lip_down_3, right_cheek_* |

**Code logic:**
```python
# Mouth openness normalized by face height
mouth_v_distance = calculate_distance(upper_lip, lower_lip) / face_height
# Typical range: 0.0 (closed) to 0.12 (wide open)
```

### Jaw (1 motor)
| Feature | Input | Maps To |
|---------|-------|---------|
| Mouth openness | mouth_v_distance | jaw |

**Code logic:**
```python
# More open mouth = lower jaw value (opens jaw)
jaw_value = map_value(lip_v, 0.0, 0.12, 2300, 2000)  # Inverted: open mouth → open jaw
```

## Value Mapping

The `map_value()` function converts normalized facial features to motor positions:

```python
def map_value(value, in_min, in_max, out_min, out_max):
    """Map value from input range to output range, clamped."""
    value = max(min(value, in_max), in_min)  # Clamp to input range
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
```

### Example Mappings

| Facial Feature | Input Range | Motor | Output Range |
|----------------|-------------|-------|--------------|
| Eyelid openness | 0.0-1.0 | left_lid | 2268-2058 (close→open) |
| Mouth openness | 0.0-0.12 | jaw | 2300-2000 (close→open) |
| Brow height | 29-33 | left_brow_1 | 2040-2414 (up→down) |
| Smile width | 0.4-0.65 | left_cheek_down | 1060-2413 (neutral→smile) |

## ROS2 Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/eye/joint_trajectory` | JointTrajectory | Eye and eyelid positions |
| `/nose_eye_brow/joint_trajectory` | JointTrajectory | Eyebrow positions |
| `/mouth/joint_trajectory` | JointTrajectory | Lip and cheek positions |
| `/jaw/joint_trajectory` | JointTrajectory | Jaw position |
| `/face_features` | Float32MultiArray | Raw features for training |

## Smoothing

Not all features use smoothing. Here's what does and doesn't use smoothing:

### Features WITH Smoothing (Rolling Average)

| Feature | Buffer | Method |
|---------|--------|--------|
| Eye gaze (h, v) | `eyes_avg` | Rolling average over 10 frames |
| Eyebrow positions | `eyes_brow_avg` | Rolling average over 10 frames |
| Mouth horizontal (smile width) | `mouth_l_h_avg`, `mouth_r_h_avg` | Rolling average + EMA (alpha=0.01) |
| Mouth vertical (lip opening) | `mouth_v_avg`, `mouth_l_v_avg`, `mouth_r_v_avg` | Rolling average + EMA (alpha=0.01) |

### Features WITHOUT Smoothing

| Feature | Reason |
|---------|--------|
| Eyelid openness | Direct mapping - needs responsive blink detection |
| Jaw | Derived from mouth_v which is already smoothed |
| Nose | Not currently controlled by mimic |
| Tongue | Not currently controlled by mimic |

### Smoothing Code

```python
# Rolling average (used for eyes, eyebrows)
self.eyes_avg = deque(maxlen=10)  # 10-frame window
self.eyes_avg.append(current_value)
smoothed = sum(self.eyes_avg) / len(self.eyes_avg)

# Exponential Moving Average + Rolling (used for mouth)
def smooth(current, previous, alpha=0.01):
    return alpha * current + (1 - alpha) * previous

mouth_distance = self.safe_smooth(self.safe_average(buffer), buffer)
```

### Smoothing Parameter

The `smoothing_window` parameter (default: 10) controls the buffer size:
```bash
ros2 launch animatronics_head_ros2 mimic_complete.launch.py smoothing_window:=5  # Faster response
```

## Launch Commands

```bash
# Complete system (hardware + camera + mimic)
ros2 launch animatronics_head_ros2 mimic_complete.launch.py camera_id:=9

# Without video preview
ros2 launch animatronics_head_ros2 mimic_complete.launch.py camera_id:=9 show_video:=false

# Feature publishing only (no motor control)
ros2 launch animatronics_head_ros2 mimic_complete.launch.py drive_motors:=false
```

## File Structure

| File | Purpose |
|------|---------|
| `face_mimic.py` | Main face tracking and motor mapping logic |
| `motor_controller.py` | Receives JointTrajectory, sends to dynamixel_controller |
| `dynamixel_controller.py` | Low-level Dynamixel SDK communication |
| `mimic_complete.launch.py` | Launches complete system |
