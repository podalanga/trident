# Trident - Mobile Robotics Competition

Autonomous line following and object manipulation system for WorldSkills competition.

## Quick Start

```bash
cd ~/Codes/ros/tn_skills/trident
./build.sh
source install/setup.zsh
ros2 launch trident_competition competition.launch.py

# Without camera (if not installed)
ros2 launch trident_competition competition.launch.py enable_camera:=false

# Start mission
ros2 topic pub --once /mission/start std_msgs/msg/Bool "data: true"
```

## Installation

```bash
# Install dependencies
sudo apt install ros-humble-realsense2-camera ros-humble-rtabmap-ros
pip3 install "numpy<2" opencv-contrib-python pyserial pyzbar

# Build
./build.sh

# Upload Arduino firmware (see src/trident_competition/arduino/)
```

## System Components

**ROS2 Nodes:**
- `mission_manager` - Coordinates Zone A & B
- `zone_a_controller` - PID line following
- `zone_b_controller` - VSLAM navigation & manipulation
- `arduino_interface` - Serial communication

**Hardware:**
- Raspberry Pi 4 + Arduino Mega 2560
- RealSense D435i camera
- 6× IR sensors, 2× N20 motors, servo gripper

## Testing

```bash
# Zone A only (no camera needed)
ros2 launch trident_competition zone_a_test.launch.py

# Monitor sensors
ros2 topic echo /arduino/ir_sensors

# Simple test (no Zone B)
ros2 launch trident_competition simple_test.launch.py
```

## Configuration

Edit `src/trident_competition/config/params.yaml` for PID tuning and navigation parameters.

## Documentation

- **Package README:** `src/trident_competition/README.md` - Full setup guide
- **Arduino:** `src/trident_competition/arduino/README.md` - Firmware & pins

## License

MIT
