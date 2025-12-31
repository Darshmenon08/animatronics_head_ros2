# Motor Calibration Reference

This document contains the calibration data and direction mappings for all 25 Dynamixel motors in the animatronics head.

## Quick Reference

- **Protocol**: Dynamixel Protocol 2.0
- **Baudrate**: 1,000,000
- **Position Range**: 0-4095 (12-bit resolution)

---

## Eyes (Port: /dev/eye, IDs: 1-6)

| Motor | ID | Min | Max | Min Position | Max Position |
|-------|-----|------|------|--------------|--------------|
| **left_v** | 1 | 2000 | 2300 | BELOW (look down) | ABOVE (look up) |
| **left_h** | 2 | 1436 | 2000 | LEFT | RIGHT |
| **right_v** | 3 | 59 | 370 | ABOVE (look up) | BELOW (look down) |
| **right_h** | 4 | 1950 | 2459 | LEFT | RIGHT |
| **left_lid** | 5 | 2058 | 2268 | OPEN | CLOSE |
| **right_lid** | 6 | 938 | 1172 | CLOSE | OPEN |

### Eye Notes
- Left and right vertical eyes move in **opposite directions** (left: max=up, right: max=down)
- Left and right horizontal eyes move in the **same direction** (max=right for both)
- Eyelids move in **opposite directions** (left: max=close, right: min=close)

---

## Eyebrows (Port: /dev/nose_eye_brow, IDs: 7-10)

| Motor | ID | Min | Max | Min Position | Max Position |
|-------|-----|------|------|--------------|--------------|
| **left_brow_1** | 7 | 2040 | 2414 | UP (raised) | DOWN (frown) |
| **left_brow_2** | 8 | 1412 | 1777 | UP (raised) | DOWN (frown) |
| **right_brow_1** | 9 | 1616 | 1907 | DOWN (frown) | UP (raised) |
| **right_brow_2** | 10 | 1440 | 1734 | DOWN (frown) | UP (raised) |

### Eyebrow Notes
- Left brows: higher value = DOWN (frown)
- Right brows: lower value = DOWN (frown)
- brow_1 = inner eyebrow, brow_2 = outer eyebrow

---

## Nose (Port: /dev/nose_eye_brow, IDs: 11-12)

| Motor | ID | Min | Max |
|-------|-----|------|------|
| **nose_left** | 11 | 1743 | 2685 |
| **nose_right** | 12 | 1722 | 2620 |

---

## Lips Upper (Port: /dev/mouth, IDs: 13-15)

| Motor | ID | Min | Max | Min Position | Max Position |
|-------|-----|------|------|--------------|--------------|
| **lip_up_1** | 13 | 1725 | 2680 | DOWN | UP |
| **lip_up_2** | 14 | 1722 | 2620 | DOWN | UP |
| **lip_up_3** | 15 | 1325 | 2680 | UP | DOWN |

### Upper Lip Notes
- lip_up_1 and lip_up_2: max = UP (higher value raises lip)
- lip_up_3: min = UP (lower value raises lip - opposite direction)

---

## Lips Lower (Port: /dev/mouth, IDs: 16-18)

| Motor | ID | Min | Max | Min Position | Max Position |
|-------|-----|------|------|--------------|--------------|
| **lip_down_1** | 16 | 1497 | 2209 | UP | DOWN |
| **lip_down_2** | 17 | 955 | 1940 | DOWN | UP |
| **lip_down_3** | 18 | 2010 | 2836 | UP | DOWN |

### Lower Lip Notes
- lip_down_1 and lip_down_3: max = DOWN (higher value lowers lip)
- lip_down_2: min = DOWN (lower value lowers lip - opposite direction)

---

## Cheeks (Port: /dev/mouth, IDs: 19-22)

| Motor | ID | Min | Max | Min Position | Max Position |
|-------|-----|------|------|--------------|--------------|
| **left_cheek_down** | 19 | 1060 | 2413 | UP | DOWN |
| **left_cheek_up** | 20 | 1555 | 2230 | UP | DOWN |
| **right_cheek_down** | 21 | 1923 | 3045 | DOWN | UP |
| **right_cheek_up** | 22 | 2037 | 2827 | DOWN | UP |

### Cheek Notes
- right_cheek_up: max = UP (higher value raises cheek)
- left_cheek_up: min = UP (lower value raises cheek)
- left_cheek_down: max = DOWN (higher value lowers cheek)
- right_cheek_down: min = DOWN (lower value lowers cheek)

---

## Tongue (Port: /dev/mouth, IDs: 23-24)

| Motor | ID | Min | Max |
|-------|-----|------|------|
| **tongue_in_out** | 23 | 1740 | 3305 |
| **tongue_left_right** | 24 | 2200 | 2826 |

---

## Jaw (Port: /dev/jaw, ID: 25)

| Motor | ID | Min | Max | Min Position | Max Position |
|-------|-----|------|------|--------------|--------------|
| **jaw** | 25 | 2000 | 2300 | CLOSED | OPEN |

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
