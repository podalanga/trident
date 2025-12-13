# IR Sensor Array Calibration Test

Standalone Arduino sketch to test, calibrate, and tare your 6 IR sensor array.

## Purpose

Before running line following, you need to:
1. **Verify all sensors work** - Check if readings change
2. **Calibrate min/max values** - Find white surface and black line readings
3. **Set threshold** - Determine the cutoff between white and black
4. **Tare sensors** - Zero out any offsets

## Hardware Setup

```
Arduino Mega → IR Sensors
Pin A8  → Sensor 0 (leftmost)
Pin A9  → Sensor 1
Pin A10 → Sensor 2
Pin A11 → Sensor 3
Pin A12 → Sensor 4
Pin A13 → Sensor 5 (rightmost)
```

**Wiring:**
- IR sensor VCC → 5V
- IR sensor GND → GND
- IR sensor OUT → Analog pin

## Upload Instructions

1. Open `ir_calibration_test.ino` in Arduino IDE
2. Select **Tools → Board → Arduino Mega 2560**
3. Select **Tools → Port** → Your Arduino port
4. Click **Upload**
5. Open **Serial Monitor** at **115200 baud**

## How to Use

### 1. Initial Test
- Upload the code
- Open Serial Monitor
- You should see continuous sensor readings:
  ```
  Raw: 234  456  789  654  321  234 | Cal: 456  678  890  ... | Line: ·····
  ```

### 2. Verify Sensors Work
- Wave your hand over sensors
- Values should change significantly
- **Good**: Values change by 200+ when hand passes
- **Bad**: Values don't change or only 1-2 sensors work

### 3. Calibration Process

**Step 1: Type 'c' and press Enter**
```
Calibration Phase 1: WHITE SURFACE
```

**Step 2: Place sensors on white paper/surface**
- Slowly move robot back and forth
- Wait 3 seconds while it records minimum values

**Step 3: Automatic transition to Phase 2**
```
Calibration Phase 2: BLACK LINE
```

**Step 4: Place sensors on black line**
- Move robot so line passes under all sensors
- Wait 3 seconds while it records maximum values

**Step 5: Results displayed automatically**
```
Sensor  | Min  | Max  | Range | Status
--------|------|------|-------|--------
   0    | 123  | 845  | 722   | Good
   1    | 134  | 856  | 722   | Good
   ...
```

### 4. Understanding Results

**Range Values:**
- **> 200**: Good sensor, strong contrast
- **100-200**: OK sensor, marginal contrast
- **< 100**: Poor sensor, check wiring or adjust height

**Threshold:**
- Automatically calculated as midpoint
- Typically around 500 on calibrated scale (0-1000)

### 5. Tare Sensors (Optional)

If sensors have drift or offset:
```
Type: t
```
This sets the current reading as zero baseline.

## Serial Commands

| Command | Action |
|---------|--------|
| `c` | Start calibration sequence |
| `t` | Tare sensors (zero current reading) |
| `r` | Reset all calibration data |
| `s` | Show detailed statistics |

## Reading the Display

### Real-time Output:
```
Raw: 234  456  789  654  321  234 | Cal: 456  678  890  ... | Line: ██·····
```

- **Raw**: Analog readings (0-1023) from ADC
- **Cal**: Calibrated values (0-1000 scale)
- **Line**: Visual indicator
  - `█` = Black line detected
  - `·` = White surface

### Statistics View (press 's'):
```
Sensor | Raw  | Min  | Max  | Calibrated | Tare | Line
-------|------|------|------|------------|------|------
   0   | 234  | 100  | 900  |    456     |  0   | NO
   1   | 678  | 120  | 920  |    789     |  0   | YES
```

## Troubleshooting

### All sensors read same value
- **Check wiring** - Verify each sensor connected to correct pin
- **Check power** - Ensure 5V connected

### Values don't change
- **Sensor height** - Too far from surface (should be 2-5mm)
- **Surface type** - Use matte black tape, not shiny
- **Lighting** - IR sensors less affected by ambient light, but check

### Only some sensors work
- **Individual wiring** - Check each analog pin connection
- **Sensor placement** - Ensure all sensors at same height
- **Dead sensor** - Replace faulty sensor

### Poor range (< 100)
- **Adjust sensor height** - Move closer to surface
- **Better contrast** - Use darker black tape
- **Clean sensors** - Dust can reduce sensitivity

## Using Calibration Data

After calibration, note these values for your line follower:

1. **Threshold** - Use in `test_firmware.ino`:
   ```cpp
   int black_threshold = 500; // From calibration
   ```

2. **Min/Max per sensor** - For advanced normalization

3. **Sensor status** - Identify weak sensors that need adjustment

## Example Calibration Session

```
> c
Calibration Phase 1: WHITE surface...
[Move robot on white paper for 3 seconds]
✓ White calibration complete!

Calibration Phase 2: BLACK line...
[Move robot over black line for 3 seconds]
✓ Black calibration complete!

CALIBRATION RESULTS
Sensor  | Min  | Max  | Range | Status
--------|------|------|-------|--------
   0    | 123  | 845  | 722   | Good
   1    | 134  | 856  | 722   | Good
   2    | 128  | 834  | 706   | Good
   3    | 131  | 841  | 710   | Good
   4    | 125  | 838  | 713   | Good
   5    | 129  | 843  | 714   | Good

Threshold: 500
Calibration saved!
```

## Next Steps

1. ✅ Verify all 6 sensors have "Good" range (> 200)
2. ✅ Note the threshold value
3. ✅ Test line detection by moving robot over line
4. ✅ Proceed to `test_firmware.ino` for full line following test
5. ✅ Transfer calibrated threshold to `params.yaml` for ROS2 system
