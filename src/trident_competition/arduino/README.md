# Arduino Firmware for Trident Competition

This folder contains the Arduino Mega firmware for the competition robot.

## Hardware Setup

### Pin Connections

**IR Sensors (6 analog sensors):**
- Sensor 0: A0
- Sensor 1: A1
- Sensor 2: A2
- Sensor 3: A3
- Sensor 4: A4
- Sensor 5: A5

**Left Motor:**
- PWM (Speed): Pin 5
- Direction 1: Pin 22
- Direction 2: Pin 23

**Right Motor:**
- PWM (Speed): Pin 6
- Direction 1: Pin 24
- Direction 2: Pin 25

**Gripper Servo:**
- Signal: Pin 9

**Buzzer:**
- Signal: Pin 8

## Installation

### Required Libraries

Install these libraries via Arduino IDE Library Manager:
1. **ArduinoJson** (by Benoit Blanchon) - Version 6.x

### Upload Instructions

1. Open `trident_firmware.ino` in Arduino IDE
2. Select **Board:** Arduino Mega 2560
3. Select **Port:** (Your Arduino's port, e.g., /dev/ttyACM0)
4. Click **Upload**

## Communication Protocol

The Arduino communicates with the Raspberry Pi via Serial at **115200 baud** using JSON messages.

### Messages from Arduino to Raspberry Pi

**IR Sensor Data** (sent at 50 Hz):
```json
{
  "type": "ir_sensors",
  "values": [s0, s1, s2, s3, s4, s5]
}
```

**Gripper Feedback:**
```json
{
  "type": "gripper_feedback",
  "action": "open" or "close",
  "position": angle
}
```

**Errors:**
```json
{
  "type": "error",
  "message": "Error description"
}
```

### Messages from Raspberry Pi to Arduino

**Motor Control:**
```json
{
  "type": "motor",
  "left_speed": 0-255,
  "right_speed": 0-255,
  "left_direction": "forward" or "backward",
  "right_direction": "forward" or "backward"
}
```

**Gripper Control:**
```json
{
  "type": "gripper",
  "action": "open" or "close"
}
```

**Buzzer Control:**
```json
{
  "type": "buzzer",
  "pattern": "single" or "double"
}
```

## Testing

Use Arduino Serial Monitor (115200 baud) to test commands manually:

```json
{"type":"motor","left_speed":150,"right_speed":150,"left_direction":"forward","right_direction":"forward"}
{"type":"gripper","action":"close"}
{"type":"buzzer","pattern":"single"}
{"type":"motor","left_speed":0,"right_speed":0,"left_direction":"forward","right_direction":"forward"}
```

## Troubleshooting

- **No serial communication:** Check baud rate (115200) and cable connection
- **Motors not responding:** Verify motor driver connections and power supply
- **IR sensors noisy:** Ensure proper grounding and check sensor alignment
- **Gripper not moving:** Check servo power supply and signal connection
