# Line Following Test Firmware

Simple standalone Arduino code for testing line following without ROS2 integration.

## Hardware Setup

### IR Sensors (6 sensors)
- Connect to analog pins A0-A5 (left to right)
- Higher values = darker surface (black line)
- Lower values = lighter surface (white background)

### Motor Driver Connections

#### Left Motor
- `AIN1` → Arduino Pin 5 (PWM)
- `AIN2` → Arduino Pin 6 (PWM)

#### Right Motor
- `BIN1` → Arduino Pin 9 (PWM)
- `BIN2` → Arduino Pin 10 (PWM)

### Motor Control Logic
- **Forward**: AIN1 = PWM value, AIN2 = 0
- **Backward**: AIN1 = 0, AIN2 = PWM value
- **Stop**: Both pins = 0

## Configuration

Edit these constants in the code to tune performance:

```cpp
float Kp = 0.8;      // Proportional gain (increase for faster correction)
float Ki = 0.01;     // Integral gain (increase to eliminate steady-state error)
float Kd = 0.3;      // Derivative gain (increase to reduce oscillations)

int baseSpeed = 150; // Base forward speed (0-255)
int maxSpeed = 200;  // Maximum speed limit (0-255)
```

## Upload Instructions

1. Open `test_firmware.ino` in Arduino IDE
2. Select **Tools → Board → Arduino Mega 2560** (or your board)
3. Select **Tools → Port** → Your Arduino port
4. Click **Upload**
5. Open **Serial Monitor** at 115200 baud to see debug output

## Serial Monitor Output

The firmware prints sensor readings and PID calculations every 200ms:

```
IR: 123  456  789  654  321  234 | Error: 0.45 | PID: 36.20 | L: 186 R: 114
```

- **IR**: Raw sensor values (6 sensors)
- **Error**: Line position error (-2.5 to +2.5)
  - Negative = line is left
  - Positive = line is right
  - Zero = line is centered
- **PID**: Control output from PID algorithm
- **L/R**: Left and right motor speeds (0-255)

## Testing Procedure

1. **Power on** the robot with Arduino connected via USB
2. **Place robot** on a track with black line on white surface
3. **Upload firmware** and open Serial Monitor
4. **Wait 2 seconds** - motors will start automatically
5. **Observe** robot following the line
6. **Tune PID** gains if needed:
   - Oscillating too much? → Decrease Kp or increase Kd
   - Not turning enough? → Increase Kp
   - Steady offset from line? → Increase Ki
   - Too slow? → Increase baseSpeed

## Pin Customization

If your motor driver uses different pins, change these definitions:

```cpp
// Change these to match your wiring
const int MOTOR_LEFT_AIN1 = 5;
const int MOTOR_LEFT_AIN2 = 6;
const int MOTOR_RIGHT_BIN1 = 9;
const int MOTOR_RIGHT_BIN2 = 10;
```

## Common Issues

### Motors not moving
- Check power supply to motor driver
- Verify motor driver connections
- Ensure PWM pins are correctly connected

### Robot drifting off line
- Calibrate IR sensor threshold (check if higher values = black)
- Adjust PID gains
- Check sensor alignment (should be evenly spaced)

### Erratic behavior
- Check sensor wiring
- Verify all 6 sensors are working (check Serial Monitor)
- Reduce baseSpeed for testing

## Difference from Main Firmware

This test firmware is **simplified** compared to `trident_firmware.ino`:
- ✅ No ROS2 JSON communication
- ✅ No gripper or buzzer control
- ✅ No state machine (always following line)
- ✅ No start/end square detection
- ✅ Direct PID tuning in code
- ✅ Continuous serial debug output

Use this for **initial hardware testing** before integrating with ROS2 system.
