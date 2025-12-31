# Motor Calibration Reference

This document contains the calibration data and direction mappings for all 25 Dynamixel motors in the animatronics head.

## Quick Reference

- **Protocol**: Dynamixel Protocol 2.0
- **Baudrate**: 1,000,000
- **Position Range**: 0-4095 (12-bit resolution)

---

## Eyes (Port: /dev/eye, IDs: 1-6)

| Motor | ID | Min | Max | Min Position | Max Position | Function |
|-------|-----|------|------|--------------|--------------|----------|
| **left_v** | 1 | 2000 | 2300 | Look DOWN | Look UP | Controls left eye vertical movement |
| **left_h** | 2 | 1436 | 2000 | Look LEFT | Look RIGHT | Controls left eye horizontal movement |
| **right_v** | 3 | 59 | 370 | Look UP | Look DOWN | Controls right eye vertical movement |
| **right_h** | 4 | 1950 | 2459 | Look LEFT | Look RIGHT | Controls right eye horizontal movement |
| **left_lid** | 5 | 2058 | 2268 | OPEN | CLOSE | Controls left eyelid open/close |
| **right_lid** | 6 | 938 | 1172 | CLOSE | OPEN | Controls right eyelid open/close |

### Eye Notes
- Left and right vertical eyes move in **opposite directions** (left: max=up, right: max=down)
- Left and right horizontal eyes move in the **same direction** (max=right for both)
- Eyelids move in **opposite directions** (left: max=close, right: min=close)

---

## Eyebrows (Port: /dev/nose_eye_brow, IDs: 7-10)

| Motor | ID | Min | Max | Min Position | Max Position | Function |
|-------|-----|------|------|--------------|--------------|----------|
| **left_brow_1** | 7 | 2040 | 2414 | UP (raised) | DOWN (frown) | Inner left eyebrow - raises/lowers inner portion |
| **left_brow_2** | 8 | 1412 | 1777 | UP (raised) | DOWN (frown) | Outer left eyebrow - raises/lowers outer portion |
| **right_brow_1** | 9 | 1616 | 1907 | DOWN (frown) | UP (raised) | Inner right eyebrow - raises/lowers inner portion |
| **right_brow_2** | 10 | 1440 | 1734 | DOWN (frown) | UP (raised) | Outer right eyebrow - raises/lowers outer portion |

### Eyebrow Notes
- Left brows: higher value = DOWN (frown)
- Right brows: higher value = UP (raised) - **opposite to left**
- brow_1 = inner eyebrow, brow_2 = outer eyebrow

---

## Nose (Port: /dev/nose_eye_brow, IDs: 11-12)

| Motor | ID | Min | Max | Function |
|-------|-----|------|------|----------|
| **nose_left** | 11 | 1743 | 2685 | Controls left nostril flare |
| **nose_right** | 12 | 1722 | 2620 | Controls right nostril flare |

---

## Lips Upper (Port: /dev/mouth, IDs: 13-15)

| Motor | ID | Min | Max | Min Position | Max Position | Function |
|-------|-----|------|------|--------------|--------------|----------|
| **lip_up_1** | 13 | 1725 | 2680 | DOWN | UP | Left side of upper lip - raises/lowers |
| **lip_up_2** | 14 | 1722 | 2620 | DOWN | UP | Center of upper lip - raises/lowers |
| **lip_up_3** | 15 | 1325 | 2680 | UP | DOWN | Right side of upper lip - raises/lowers |

### Upper Lip Notes
- lip_up_1 and lip_up_2: max value = UP (raises lip)
- lip_up_3: min value = UP (opposite direction)

---

## Lips Lower (Port: /dev/mouth, IDs: 16-18)

| Motor | ID | Min | Max | Min Position | Max Position | Function |
|-------|-----|------|------|--------------|--------------|----------|
| **lip_down_1** | 16 | 1497 | 2209 | UP | DOWN | Left side of lower lip - raises/lowers |
| **lip_down_2** | 17 | 955 | 1940 | DOWN | UP | Center of lower lip - raises/lowers |
| **lip_down_3** | 18 | 2010 | 2836 | UP | DOWN | Right side of lower lip - raises/lowers |

### Lower Lip Notes
- lip_down_1 and lip_down_3: max value = DOWN (lowers lip)
- lip_down_2: min value = DOWN (opposite direction)

---

## Cheeks (Port: /dev/mouth, IDs: 19-22)

| Motor | ID | Min | Max | Min Position | Max Position | Function |
|-------|-----|------|------|--------------|--------------|----------|
| **left_cheek_down** | 19 | 1060 | 2413 | SMILE | FROWN | Pulls left lower cheek for expressions |
| **left_cheek_up** | 20 | 1555 | 2230 | UP (smile) | DOWN | Pushes left upper cheek up when smiling |
| **right_cheek_down** | 21 | 1923 | 3045 | FROWN | SMILE | Pulls right lower cheek for expressions |
| **right_cheek_up** | 22 | 2037 | 2827 | DOWN | UP (smile) | Pushes right upper cheek up when smiling |

### Cheek Notes
- **Smiling**: left_cheek_down=MIN (1060), right_cheek_down=MAX (3045), both cheek_up go UP
- **Frowning**: left_cheek_down=MAX (2413), right_cheek_down=MIN (1923)
- Left and right cheeks move in **opposite directions** (mirrored)

---

## Tongue (Port: /dev/mouth, IDs: 23-24)

| Motor | ID | Min | Max | Function |
|-------|-----|------|------|----------|
| **tongue_in_out** | 23 | 1740 | 3305 | Extends/retracts tongue |
| **tongue_left_right** | 24 | 2200 | 2826 | Moves tongue left/right |

---

## Jaw (Port: /dev/jaw, ID: 25)

| Motor | ID | Min | Max | Min Position | Max Position | Function |
|-------|-----|------|------|--------------|--------------|----------|
| **jaw** | 25 | 2000 | 2300 | OPEN | CLOSED | Opens and closes the mouth |

---

## Port Assignments

| Port | Device Path | Motors |
|------|-------------|--------|
| **eye** | `/dev/eye` | IDs 1-6 (eyes, lids) |
| **nose_eye_brow** | `/dev/nose_eye_brow` | IDs 7-12 (eyebrows, nose) |
| **mouth** | `/dev/mouth` | IDs 13-24 (lips, cheeks, tongue) |
| **jaw** | `/dev/jaw` | ID 25 (jaw) |

---

## Calibration Date

Last calibrated: 2025-12-31
