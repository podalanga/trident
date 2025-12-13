Optimizing tool selection...I'll provide a detailed breakdown of every file in the codebase. Let me start by reading through each component systematically. 

---

# **Complete Codebase Breakdown - Piece by Piece**

## **📦 Package Structure Files**

### **1. package.xml** - ROS2 Package Manifest
```xml
<package format="3">
  <name>trident_competition</name>
  <version>1.0.0</version>
```
**Purpose:** Defines the ROS2 package metadata and dependencies
- **Lines 4-7:** Package identification (name, version, description, maintainer)
- **Lines 9-10:** Build system requirements (`ament_cmake`, `ament_cmake_python`)
- **Lines 12-23:** Core ROS2 dependencies
  - `rclpy`: Python ROS2 client library
  - `rclcpp`: C++ ROS2 client library
  - `std_msgs`, `sensor_msgs`, `geometry_msgs`: Message types
  - `tf2`: Transformation library
  - `cv_bridge`: OpenCV-ROS bridge
- **Lines 25-27:** Hardware/algorithm packages
  - `realsense2_camera`: Intel RealSense driver
  - `rtabmap_ros`: VSLAM library
- **Lines 29-31:** Python runtime dependencies

---

### **2. CMakeLists.txt** - Build Configuration
```cmake
cmake_minimum_required(VERSION 3.8)
project(trident_competition)
```
**Purpose:** Configures how the package is built and installed
- **Lines 1-2:** CMake version and project name
- **Lines 4-6:** Compiler warnings for code quality
- **Lines 8-12:** Find ROS2 dependencies
- **Line 14:** Install Python package module (`trident_competition/`)
- **Lines 16-21:** Mark Python scripts as executables
  - Makes `.py` files executable from ROS2 commands
- **Lines 23-37:** Install data files (launch, config, arduino firmware)
- **Lines 39-42:** Enable testing framework

---

### **3. setup.py** - Python Package Setup
```python
from setuptools import setup
```
**Purpose:** Python setuptools configuration for the package
- **Lines 3-5:** Package metadata
- **Lines 6-11:** Data files registration
  - Registers package with ROS2 index
  - Includes package.xml
- **Lines 12-22:** Standard Python package metadata
  - No console_scripts since executables handled by CMake

---

## **⚙️ Configuration Files**

### **4. `config/params.yaml`** - ROS2 Parameters
```yaml
zone_a_controller:
  ros__parameters:
    kp: 0.8
```
**Purpose:** Runtime-tunable parameters for all nodes

**Zone A Section (Lines 2-16):**
- **PID Gains (Lines 4-6):**
  - `kp: 0.8` - Proportional gain (how aggressively to correct errors)
  - `ki: 0.01` - Integral gain (corrects steady-state errors)
  - `kd: 0.3` - Derivative gain (dampens oscillations)
- **Motor Speeds (Lines 8-10):**
  - `base_speed: 150` - Normal forward speed (PWM 0-255)
  - `max_speed: 200` - Maximum speed limit
- **Control Rate (Line 13):** 50 Hz control loop
- **IR Thresholds (Lines 15-16):**
  - `black_threshold: 800` - Minimum value to detect black line
  - `white_threshold: 200` - Maximum value for white surface

**Zone B Section (Lines 18-36):**
- **VSLAM (Lines 21-22):**
  - `rotation_speed: 0.1` - rad/s for 360° mapping (slow = better accuracy)
  - `vslam_update_rate: 30.0` - Processing frequency
- **Navigation Distances (Lines 24-26):**
  - `approach_distance_qr: 0.2` - Stop 200mm from North wall
  - `approach_distance_drop: 0.1` - Stop 100mm from East wall
- **Tolerances (Lines 28-30):**
  - `navigation_tolerance: 0.05` - Accept ±50mm positioning error
  - `orientation_tolerance: 0.087` - Accept ±5° rotation error
- **Vision (Lines 32-34):** Number of detection attempts before failure
- **Gripper (Line 37):** Delay for servo movement

**Arduino Section (Lines 39-43):**
- Serial port, baud rate, timeout configuration

**Mission Manager Section (Lines 45-48):**
- Maximum mission time and status update frequency

---

### **5. `config/params.py`** - Python Constants
```python
LINE_FOLLOW_PID = {
    'kp': 0.8,
    'ki': 0.01,
    'kd': 0.3
}
```
**Purpose:** Hardcoded constants accessible from Python code (alternative to YAML)

**Lines 3-9:** Zone A PID constants (duplicates YAML for code access)  
**Lines 11-13:** Motor speed limits  
**Lines 15-17:** VSLAM rotation and update rates  
**Lines 19-21:** Wall standoff distances for navigation  
**Lines 23-25:** Navigation tolerance thresholds  
**Lines 27-31:** Color detection parameters
- `MIN_SQUARE_AREA`, `MAX_SQUARE_AREA` - Expected pixel size of 100x100mm squares
**Lines 33-36:** QR detection parameters
- Min/max detection distance range
**Lines 38-41:** Gripper servo angles and timing  
**Lines 43-45:** Buzzer timing patterns  
**Lines 47-50:** Safety limits (battery, time, clearance)  
**Lines 52-55:** Serial communication settings  
**Lines 57-60:** Camera topic names  
**Lines 62-64:** VSLAM topic names

---

## ** ROS2 Nodes - The Brain**

### **6. `scripts/mission_manager.py`** - Mission Coordinator

```python
class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
```

**Purpose:** High-level orchestrator coordinating Zone A and Zone B

**Initialization (`__init__`, Lines 11-37):**
- **Lines 14-16:** State tracking flags
  - `zone_a_complete` - Has Zone A finished?
  - `zone_b_complete` - Has Zone B finished?
  - `mission_start_time` - Timestamp for duration calculation
- **Lines 18-20:** Publishers to start zones
  - `/zone_a/start` - Triggers Zone A controller
  - `/zone_b/start` - Triggers Zone B controller
- **Lines 22-29:** Subscribers to monitor progress
  - `/zone_a/state` - Listens for Zone A completion
  - `/zone_b/state` - Listens for Zone B completion
  - `/mission/start` - External trigger to begin mission
- **Line 32:** 5-second status timer

**Mission Start (`mission_start_callback`, Lines 39-48):**
- **Line 40:** Check if mission not already running
- **Lines 41-44:** Log mission start banner
- **Line 45:** Record start timestamp
- **Line 48:** Call `start_zone_a()` to begin

**Zone A Monitoring (`zone_a_state_callback`, Lines 50-62):**
- **Line 54:** Detect 'complete' state message
- **Lines 55-59:** Log Zone A completion with elapsed time
- **Line 62:** Start Zone B after 2-second pause (robot positioning)

**Zone B Monitoring (`zone_b_state_callback`, Lines 64-73):**
- **Line 67:** Detect Zone B completion
- **Lines 68-72:** Log total mission time
- **Line 73:** Trigger mission completion handler

**Control Functions (Lines 75-88):**
- `start_zone_a()` - Publishes Bool(True) to `/zone_a/start`
- `start_zone_b()` - Publishes Bool(True) to `/zone_b/start`
- `complete_mission()` - Generates final mission report

**Reporting (Lines 90-100):**
- `generate_mission_report()` - Creates JSON report with completion status
- `publish_status()` - Periodic 5s updates with checkmarks (○ vs ✓)

**Main Function (Lines 103-113):**
- Initializes ROS2, creates node, spins (handles callbacks)
- Cleanup on Ctrl+C

---

### **7. `scripts/zone_a_controller.py`** - Line Following

```python
class ZoneAController(Node):
    def __init__(self):
        super().__init__('zone_a_controller')
```

**Purpose:** PID-based line following using 6 IR sensors

**Initialization (`__init__`, Lines 16-69):**
- **Lines 18-35:** Parameter declaration and retrieval
  - Loads PID gains (kp, ki, kd) from params.yaml
  - Motor speeds: `base_speed=150`, `max_speed=200`
  - Control rate: 50 Hz (20ms period)
- **Lines 37-39:** PID controller instantiation
  - Creates `PIDController` object with gains
  - Calculates `dt = 1/50 = 0.02 seconds`
- **Lines 41-43:** State machine initialization
  - States: `idle` → `waiting_start` → `line_following` → `aligning` → `complete`
  - `sensor_values` - Array for 6 IR readings
- **Lines 45-48:** Publishers
  - `/arduino/motor_command` - Motor control JSON
  - `/arduino/buzzer_command` - Buzzer patterns
  - `/zone_a/state` - Current state for mission_manager
- **Lines 50-60:** Subscribers
  - `/arduino/ir_sensors` - Receives sensor array at 50Hz
  - `/zone_a/start` - Listens for start signal
- **Line 63:** Control timer at 50Hz

**Sensor Callback (`ir_callback`, Lines 71-73):**
- Stores latest sensor readings from Arduino
- `msg.data` contains 6 integers (0-1023)

**Start Handler (`start_callback`, Lines 75-78):**
- Transitions from `idle` to `waiting_start` when mission begins

**Start Square Detection (`check_start_square`, Lines 80-83):**
- Returns `True` if all 6 sensors > 800 (all black)
- Indicates robot is in start square

**Control Loop (`control_loop`, Lines 85-122):**
The heart of Zone A - runs every 20ms

- **Lines 87-89:** Publish current state to mission manager
- **Lines 91-95:** `waiting_start` state
  - Waits for start square detection
  - Transitions to `line_following`
  - Resets PID to clear integral windup
- **Lines 97-117:** `line_following` state (the main loop)
  - **Lines 99-103:** End square detection
    - If 5+ sensors see black → end reached
    - Stop motors, beep, transition to `aligning`
  - **Line 106:** Calculate line position error
    - Uses weighted average of sensor array
  - **Line 109:** Compute PID correction
  - **Lines 112-113:** Calculate differential steering
    - `left_speed = base + correction` (turn right if positive)
    - `right_speed = base - correction` (turn left if negative)
  - **Lines 116-117:** Clamp speeds to [0, max_speed]
  - **Line 120:** Send motor command to Arduino
- **Lines 122-124:** `aligning` state
  - Position robot at right edge of end square
  - Prepares for Zone B entry (facing North wall)

**Alignment (`align_for_zone_b`, Lines 126-136):**
- Simplified implementation: 1-second delay then mark complete
- Production version would use sensors to align to edge

**Motor Commands (Lines 138-151):**
- `send_motor_command()` - Creates JSON motor command
  - Format: `{"type":"motor", "left_speed":150, "right_speed":150, ...}`
- `stop_motors()` - Sets both speeds to 0

**Buzzer (Lines 153-160):**
- `trigger_buzzer()` - Sends JSON buzzer pattern
  - Patterns: "single", "double"

**Main (Lines 163-176):**
- Ensures motors stop on shutdown

---

### **8. `trident_competition/pid_controller.py`** - PID Algorithm

```python
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
```

**Purpose:** PID control algorithm for line following

**Initialization (`__init__`, Lines 5-18):**
- **Lines 13-15:** Store PID gains
- **Lines 16-18:** Initialize state variables
  - `previous_error` - For derivative calculation
  - `integral` - Accumulated error over time
  - `max_integral` - Anti-windup limit (prevents integral from growing unbounded)

**PID Computation (`compute`, Lines 20-50):**
- **Line 31:** **Integral term** (accumulates error over time)
  - `self.integral += error * dt`
  - Corrects steady-state errors (e.g., constant offset)
- **Line 32:** **Anti-windup** clamping
  - Prevents integral from growing too large during prolonged errors
  - Range: [-100, 100]
- **Lines 35-38:** **Derivative term** (rate of change)
  - `derivative = (error - previous_error) / dt`
  - Dampens oscillations, predicts future error
- **Line 41:** **PID output formula**
  - `output = Kp*error + Ki*integral + Kd*derivative`
- **Lines 43-44:** Store error for next iteration

**Reset (`reset`, Lines 46-49):**
- Clears integral and previous_error
- Called when transitioning states to prevent carryover

**Line Position Calculation (`calculate_line_position_error`, Lines 52-72):**
```python
def calculate_line_position_error(sensor_values: list) -> float:
```
- **Purpose:** Converts 6 sensor readings into line position error

- **Line 60:** Sum all sensor values
- **Lines 62-63:** If total < 100, no line detected (return 0)
- **Line 66:** **Weighted average position**
  - Formula: `Σ(sensor[i] * i) / Σ(sensor[i])`
  - Sensor indices: 0, 1, 2, 3, 4, 5
  - Result: 0-5 (left to right)
- **Line 70:** **Center error calculation**
  - Center position = 2.5 (between sensors 2 and 3)
  - Error = position - 2.5
  - Negative = line is left, Positive = line is right

**End Square Detection (`detect_end_square`, Lines 75-87):**
```python
def detect_end_square(sensor_values: list, threshold: int = 800) -> bool:
```
- Counts sensors detecting black (value > 800)
- Returns `True` if 5+ sensors see black
- Indicates robot entered end square

---

### **9. `scripts/arduino_interface.py`** - Serial Bridge

```python
class ArduinoInterface(Node):
    def __init__(self):
        super().__init__('arduino_interface')
```

**Purpose:** Bidirectional JSON communication between ROS2 and Arduino

**Initialization (`__init__`, Lines 16-62):**
- **Lines 18-25:** Parameter loading
  - Serial port: `/dev/ttyACM0`
  - Baud rate: 115200
  - Timeout: 1.0 second
- **Lines 27-37:** **Serial port connection**
  - Opens serial port with specified parameters
  - Error handling if Arduino not connected
- **Lines 39-41:** **Arduino → ROS publishers**
  - `/arduino/ir_sensors` - IR sensor array (Int32MultiArray)
  - `/arduino/gripper_feedback` - Gripper status (String JSON)
- **Lines 43-49:** **ROS → Arduino subscribers**
  - `/arduino/motor_command` - Motor control
  - `/arduino/gripper_command` - Gripper open/close
  - `/arduino/buzzer_command` - Buzzer patterns
- **Lines 51-55:** **Background reading thread**
  - Separate thread continuously reads serial port
  - Daemon thread (dies with main program)

**Command Handlers (Lines 64-81):**
- `motor_command_callback()` - Parse JSON, forward to Arduino
- `gripper_command_callback()` - Parse JSON, forward to Arduino
- `buzzer_command_callback()` - Parse JSON, forward to Arduino
- All follow pattern: Receive ROS message → Parse JSON → Call `send_to_arduino()`

**Arduino Communication (`send_to_arduino`, Lines 83-95):**
```python
def send_to_arduino(self, data: dict):
    json_str = json.dumps(data) + '\n'
    self.serial_port.write(json_str.encode('utf-8'))
```
- Converts Python dict to JSON string
- Adds newline delimiter
- Encodes to UTF-8 bytes and sends

**Serial Reading Thread (`read_serial_loop`, Lines 97-115):**
```python
def read_serial_loop(self):
    while self.running and rclpy.ok():
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline()
            data = json.loads(line)
            self.process_arduino_data(data)
```
- **Runs continuously in background thread**
- **Line 99:** Check if data available
- **Line 100:** Read until newline character
- **Lines 103-106:** Parse JSON and route to handler
- **Lines 107-108:** Warn on invalid JSON

**Data Processing (`process_arduino_data`, Lines 117-135):**
- **Lines 121-125:** Handle IR sensor data
  - Extract `values` array
  - Publish to `/arduino/ir_sensors`
- **Lines 127-131:** Handle gripper feedback
  - Forward JSON status to ROS topic

**Cleanup (`destroy_node`, Lines 137-143):**
- Stops reading thread
- Closes serial port

---

### **10. `scripts/zone_b_controller.py`** - VSLAM Navigation

```python
class ZoneBController(Node):
    def __init__(self):
        super().__init__('zone_b_controller')
```

**Purpose:** Computer vision-based navigation and object manipulation

**Initialization (`__init__`, Lines 18-98):**
- **Lines 20-31:** Parameter loading
  - Rotation speed, approach distances, tolerances
- **Lines 33-36:** **State machine**
  - States: `idle` → `initializing_vslam` → `mapping` → `detecting_colors` → `detecting_qrs` → `executing_mission` → `complete`
  - `current_qr` - Which QR being processed (QR1, QR2, QR3)
  - `mission_phase` - Sub-state within mission execution
- **Lines 38-46:** **Vision components**
  - `CvBridge` - Converts ROS images to OpenCV
  - `camera_intrinsics` - Camera calibration (fx, fy, cx, cy)
  - `ColorDetector`, `QRDetector` - Vision algorithms
  - `rgb_image`, `depth_image` - Latest camera frames
- **Lines 48-53:** **Arena map data structures**
  - `east_wall_colors` - {ColorPos1: {color, position}, ...}
  - `north_wall_qrs` - {QR1: {text, position}, ...}
  - `qr_to_drop_mapping` - Which QR goes to which color
- **Lines 55-57:** **VSLAM state**
  - `current_pose` - Robot location from RTAB-Map
  - `vslam_initialized` - Has VSLAM started?
- **Lines 59-64:** **Publishers**
  - `/cmd_vel` - Velocity commands for navigation
  - `/arduino/gripper_command` - Gripper control
  - `/arduino/buzzer_command` - Audio feedback
  - `/zone_b/state` - Status for mission manager
- **Lines 66-79:** **Subscribers**
  - `/camera/color/image_raw` - RGB frames
  - `/camera/depth/image_rect_raw` - Depth frames
  - `/camera/color/camera_info` - Camera calibration
  - `/rtabmap/localization_pose` - Robot pose from VSLAM
  - `/zone_b/start` - Start signal

**Camera Callbacks (Lines 100-117):**
- `rgb_callback()` - Converts ROS Image to OpenCV BGR
- `depth_callback()` - Converts ROS Image to OpenCV depth array
- `camera_info_callback()` - Extracts camera intrinsics
  - **Lines 111-116:** Creates vision detectors once intrinsics received

**VSLAM Callback (`pose_callback`, Lines 119-123):**
- Stores latest robot pose
- Sets `vslam_initialized` flag on first pose

**Control Loop (`control_loop`, Lines 131-156):**
Main state machine - runs at 10 Hz

- **Lines 133-135:** Publish current state
- **Lines 137-140:** `initializing_vslam` - Wait for VSLAM ready
- **Line 143:** `mapping` - Execute 360° rotation
- **Line 146:** `detecting_colors` - Find colored squares
- **Line 149:** `detecting_qrs` - Find QR codes
- **Lines 151-153:** `executing_mission` - Pickup/delivery loop

**360° Mapping (`perform_360_mapping`, Lines 158-176):**
```python
def perform_360_mapping(self):
    twist = Twist()
    twist.angular.z = self.rotation_speed  # 0.1 rad/s
    duration = 2 * π / 0.1 ≈ 63 seconds
```
- Rotates slowly for RTAB-Map to build 3D map
- Duration = 2π / rotation_speed
- RTAB-Map processes RGB-D frames during rotation

**Color Detection (`detect_east_wall_colors`, Lines 178-198):**
```python
def detect_east_wall_colors(self):
    detections = self.color_detector.detect_colored_squares(
        self.rgb_image, self.depth_image)
```
- **Line 184:** Call vision detector
- **Lines 186-195:** When 3 colors found:
  - Sort by Y-coordinate (North to South)
  - Assign ColorPos1, ColorPos2, ColorPos3
- **Line 198:** Retry if not all 3 detected

**QR Detection (`detect_north_wall_qrs`, Lines 200-223):**
```python
def detect_north_wall_qrs(self):
    detections = self.qr_detector.detect_qr_codes(
        self.rgb_image, self.depth_image)
```
- **Line 206:** Call QR detector
- **Lines 208-217:** When 3 QRs found:
  - Sort by X-coordinate (West to East)
  - Assign QR1, QR2, QR3
  - Create QR-to-color mapping
- **Lines 219-222:** Start mission execution

**QR-Color Mapping (`create_qr_color_mapping`, Lines 225-237):**
```python
def create_qr_color_mapping(self):
    qr_text = qr_data['text']  # e.g., "tn-red"
    target_color = qr_text.replace('tn-', '')  # "red"
```
- Extracts color name from QR text
- Matches to ColorPos location
- Creates lookup table for delivery

**Mission Execution Loop (`execute_manipulation_loop`, Lines 239-256):**
State machine within state machine:
- `navigate_to_qr` → `align_pickup` → `pickup` → `navigate_to_drop` → `align_drop` → `drop`
- Cycles through QR1, QR2, QR3

**Navigation Phases (Lines 258-329):**
- `navigate_to_qr()` - Move to QR pickup location (200mm offset)
- `align_for_pickup()` - Visual servoing to center on QR
- `pickup_coin()` - Close gripper, lift
- `navigate_to_drop()` - Move to color square (100mm offset)
- `align_for_drop()` - Visual servoing to center on color
- `drop_coin()` - Open gripper, move to next QR

---

### **11. `trident_competition/vision_detection.py`** - Computer Vision

#### **ColorDetector Class (Lines 9-150)**

```python
class ColorDetector:
    COLOR_RANGES = {
        'red': [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))],
        'green': [((40, 50, 50), (80, 255, 255))],
        'blue': [((100, 50, 50), (130, 255, 255))]
    }
```

**HSV Color Ranges (Lines 11-21):**
- **Red:** Two ranges (H wraps around at 180°)
  - Range 1: Hue 0-10 (reddish-orange)
  - Range 2: Hue 170-180 (purplish-red)
- **Green:** Hue 40-80
- **Blue:** Hue 100-130
- S (Saturation) > 50 ensures vivid colors
- V (Value) > 50 ensures bright colors

**Initialization (`__init__`, Lines 23-30):**
- Stores camera intrinsics (fx, fy, cx, cy)
- Creates 5x5 elliptical morphological kernel

**Main Detection (`detect_colored_squares`, Lines 32-78):**
```python
def detect_colored_squares(self, rgb_image, depth_image):
```
- **Line 47:** Convert BGR → HSV color space
- **Lines 50-75:** **For each color:**
  - **Line 51:** Create binary mask using color ranges
  - **Line 54:** Find contours in mask
  - **Lines 56-75:** **For each contour:**
    - **Line 57:** Calculate pixel area
    - **Line 60:** Validate area matches 100x100mm square at depth
    - **Lines 61-68:** Check shape (aspect ratio 0.8-1.2)
    - **Lines 65-66:** Calculate centroid (cx, cy)
    - **Lines 69-71:** Get depth and calculate 3D position
    - **Lines 73-79:** Store detection data

**Color Masking (`_create_color_mask`, Lines 80-94):**
```python
def _create_color_mask(self, hsv_image, ranges):
```
- **Lines 85-89:** Apply each HSV range, combine with OR
- **Lines 92-93:** Morphological opening (removes noise)
- **Line 93:** Morphological closing (fills holes)

**Area Validation (`_is_valid_square_area`, Lines 96-124):**
```python
def _is_valid_square_area(self, pixel_area, depth_image, contour):
```
- **Purpose:** Verify contour is correct physical size

- **Lines 98-100:** Create mask from contour
- **Lines 102-107:** Get median depth of contour
- **Lines 112-114:** **Calculate expected pixel area**
  - Real square size: 0.1m × 0.1m
  - Projected width: `(0.1 * fx) / depth`
  - Projected height: `(0.1 * fy) / depth`
  - Expected area = width × height
- **Line 117:** Allow ±30% tolerance

**3D Projection (`_deproject_pixel`, Lines 126-143):**
```python
def _deproject_pixel(self, x, y, depth):
```
- **Pinhole camera model:**
  - `X = (x - cx) * Z / fx`
  - `Y = (y - cy) * Z / fy`
  - `Z = depth / 1000` (convert mm to m)
- Returns (X, Y, Z) in camera frame

#### **QRDetector Class (Lines 153-243)**

```python
class QRDetector:
    def __init__(self, camera_intrinsics):
```

**Initialization (Lines 153-163):**
- Stores camera intrinsics
- Creates OpenCV QR detector object

**QR Detection (`detect_qr_codes`, Lines 165-229):**
```python
def detect_qr_codes(self, rgb_image, depth_image):
```

**Primary Method - OpenCV (Lines 180-200):**
- **Line 180:** `detectAndDecodeMulti()` - Detects all QRs at once
- **Lines 182-197:** **For each detected QR:**
  - **Lines 186-188:** Calculate centroid from corner points
  - **Lines 191-193:** Get depth at centroid
  - **Lines 196-200:** Store QR data (text, position, corners)

**Fallback Method - pyzbar (Lines 202-224):**
- If OpenCV fails, use pyzbar library
- **Line 203:** `pyzbar.decode()` - Alternative QR decoder
- **Lines 204-223:** Similar processing as OpenCV method

**3D Projection (Lines 231-243):**
- Same pinhole camera math as ColorDetector

#### **Visualization Function (Lines 246-276)**

```python
def visualize_detections(image, color_detections, qr_detections):
```
- Draws colored rectangles around detected squares
- Draws yellow polygons around QR codes
- Adds text labels
- Returns annotated image for debugging

---

### **12. `arduino/trident_firmware/trident_firmware.ino`** - Arduino Firmware

#### **Pin Definitions (Lines 14-30)**

```cpp
const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5};
```
- **IR Sensors (Line 16):** 6 analog pins for line sensors
- **Motors (Lines 18-24):**
  - PWM pins (5, 6) control speed via analogWrite(0-255)
  - DIR pins (22-25) control direction via digitalWrite(H/L)
  - L298N H-bridge wiring: DIR1=HIGH, DIR2=LOW → Forward
- **Gripper (Line 27):** Servo on pin 9
- **Buzzer (Line 30):** Digital pin 8

#### **Constants (Lines 32-38)**

- `SERIAL_BAUD: 115200` - Match ROS side
- `IR_SEND_RATE: 50 Hz` - Send sensor data every 20ms
- `GRIPPER_CLOSE_ANGLE: 30°` - Closed position
- `GRIPPER_OPEN_ANGLE: 90°` - Open position
- `BUZZER_DURATION: 300ms` - Beep length

#### **Setup Function (Lines 45-76)**

```cpp
void setup() {
  Serial.begin(SERIAL_BAUD);
```
- **Line 47:** Initialize serial at 115200 baud
- **Lines 49-52:** Configure IR sensor pins as INPUT
- **Lines 54-59:** Configure motor pins as OUTPUT
- **Lines 61-63:** **Attach gripper servo**
  - Initialize to OPEN position
- **Lines 65-67:** Configure buzzer pin
- **Line 70:** Startup beep to confirm Arduino running

#### **Main Loop (Lines 78-91)**

```cpp
void loop() {
  // Read IR sensors at 50Hz
  if (millis() - lastIRSendTime >= 20) {
    readAndSendIRSensors();
  }
  
  // Process incoming commands
  if (Serial.available()) {
    processSerialCommand();
  }
}
```
- **Lines 80-84:** **IR sensor publishing at 50 Hz**
  - Check if 20ms elapsed
  - Read all 6 sensors
  - Send as JSON
- **Lines 86-89:** **Command processing**
  - Check if serial data available
  - Parse and execute command

#### **IR Sensor Functions (Lines 93-110)**

```cpp
void readAndSendIRSensors() {
  for (int i = 0; i < 6; i++) {
    irSensorValues[i] = analogRead(IR_PINS[i]);
  }
```
- **Lines 96-98:** Read all 6 analog values (0-1023)
- **Lines 101-108:** **Build JSON message:**
  ```json
  {"type":"ir_sensors","values":[234,567,891,345,678,912]}
  ```
- **Lines 107-108:** Serialize and send with newline

#### **Serial Command Processing (Lines 112-137)**

```cpp
void processSerialCommand() {
  String jsonString = Serial.readStringUntil('\n');
  StaticJsonDocument<300> doc;
  deserializeJson(doc, jsonString);
```
- **Line 113:** Read until newline character
- **Lines 115-121:** Parse JSON using ArduinoJson library
- **Lines 124-134:** **Route by message type:**
  - `"motor"` → `handleMotorCommand()`
  - `"gripper"` → `handleGripperCommand()`
  - `"buzzer"` → `handleBuzzerCommand()`

#### **Motor Control (Lines 147-178)**

```cpp
void handleMotorCommand(JsonDocument& doc) {
  int leftSpeed = doc["left_speed"];
  int rightSpeed = doc["right_speed"];
  const char* leftDir = doc["left_direction"];
  const char* rightDir = doc["right_direction"];
```
- **Lines 148-151:** Extract command parameters
- **Lines 154-158:** Set left and right motors
- **Function `setMotor()` (Lines 160-176):**
  - **Line 162:** Clamp speed to 0-255
  - **Lines 165-171:** Set direction pins
    - Forward: DIR1=HIGH, DIR2=LOW
    - Backward: DIR1=LOW, DIR2=HIGH
  - **Line 174:** Set PWM speed with `analogWrite()`

#### **Gripper Control (Lines 180-200)**

```cpp
void handleGripperCommand(JsonDocument& doc) {
  const char* action = doc["action"];
  
  if (strcmp(action, "open") == 0) {
    gripperServo.write(GRIPPER_OPEN_ANGLE);
  }
  else if (strcmp(action, "close") == 0) {
    gripperServo.write(GRIPPER_CLOSE_ANGLE);
  }
```
- **Lines 183-187:** Set servo angle based on action
- **Function `sendGripperFeedback()` (Lines 189-198):**
  - Sends confirmation back to ROS
  ```json
  {"type":"gripper_feedback","action":"close","position":30}
  ```

#### **Buzzer Control (Lines 202-223)**

```cpp
void handleBuzzerCommand(JsonDocument& doc) {
  const char* pattern = doc["pattern"];
  
  if (strcmp(pattern, "single") == 0) {
    beep(1);
  }
  else if (strcmp(pattern, "double") == 0) {
    beep(2);
  }
}
```
- **Lines 203-210:** Parse pattern and call `beep()`
- **Function `beep()` (Lines 212-223):**
  - **Line 213:** Loop for number of beeps
  - **Lines 214-216:** Turn buzzer ON for BUZZER_DURATION
  - **Lines 218-220:** Pause between beeps

---

## **🚀 Launch Files**

### **13. `launch/competition.launch.py`** - Main Launch

```python
def generate_launch_description():
```

**Purpose:** Launches complete mission (all nodes + camera + VSLAM)

**Lines 15-23:** Get parameter file path  
**Lines 25-35:** **Launch arguments:**
- `use_sim_time` - For simulation compatibility
- `enable_camera` - Can disable if hardware not connected

**Lines 37-73:** **ROS2 Node declarations:**
- Mission Manager (Lines 37-43)
- Zone A Controller (Lines 45-51)
- Zone B Controller (Lines 53-59)
- Arduino Interface (Lines 61-67)

**Lines 69-91:** **RealSense Camera Launch:**
- Conditional (only if `enable_camera:=true`)
- Includes `rs_launch.py` from realsense2_camera package
- **Launch arguments (Lines 77-86):**
  - `enable_depth: true` - Depth stream
  - `enable_color: true` - RGB stream
  - `align_depth.enable: true` - Align depth to RGB
  - `enable_gyro/accel: true` - IMU data
  - `unite_imu_method: 1` - Combine IMU streams

**Lines 93-106:** Return LaunchDescription with all components

---

### **14. `launch/rtabmap.launch.py`** - VSLAM

```python
def generate_launch_description():
```

**Purpose:** Launches RTAB-Map VSLAM system

**Lines 25-48:** **RTAB-Map Parameters:**
- **Basic (Lines 26-31):**
  - `frame_id: 'base_link'` - Robot base frame
  - `subscribe_depth/rgb: True` - Use RGB-D data
  - `approx_sync: True` - Tolerate timestamp misalignment
- **Memory (Lines 34-35):**
  - `IncrementalMemory: true` - Build map online
  - `InitWMWithAllNodes: false` - Don't load all to working memory
- **RGBD (Lines 36-41):**
  - `NeighborLinkRefining: true` - Refine loop closures
  - `AngularUpdate: 0.01` - Update every 0.01 radians
  - `LinearUpdate: 0.01` - Update every 0.01 meters
- **Grid (Line 42):** `FromDepth: true` - Build occupancy grid from depth
- **Registration (Lines 43-44):**
  - `Force3DoF: true` - Constrain to 2D plane
  - `Strategy: 1` - Use ICP (Iterative Closest Point)
- **Odometry (Lines 46-48):**
  - Visual odometry parameters

**Lines 50-65:** **RTAB-Map Main Node:**
- Executable: `rtabmap`
- **Remappings (Lines 58-62):** Connect to camera topics
- **Argument (Line 63):** `--delete_db_on_start` - Fresh map each run

**Lines 67-78:** **RTAB-Map Visual Odometry:**
- Executable: `rgbd_odometry`
- Estimates motion from RGB-D frames

**Lines 80-93:** **RTAB-Map Visualization:**
- Executable: `rtabmapviz`
- 3D viewer for map and trajectory

---

## **📊 Summary: How It All Works Together**

### **Data Flow Diagram:**

```
┌─────────────────────┐
│  Mission Manager    │ ← Start signal
│   (Coordinator)     │ → Starts Zone A, then Zone B
└─────────┬───────────┘
          │
    ┌─────┴──────┐
    ▼            ▼
┌─────────┐  ┌─────────┐
│ Zone A  │  │ Zone B  │
│ (Line)  │  │ (VSLAM) │
└────┬────┘  └────┬────┘
     │            │
     │            ├─────► ColorDetector ──► HSV masking
     │            │                          ├─► Red squares
     │            │                          ├─► Green squares
     │            │                          └─► Blue squares
     │            │
     │            ├─────► QRDetector ────► OpenCV/pyzbar
     │            │                          └─► QR1, QR2, QR3
     │            │
     │            └─────► RTAB-Map ──────► 3D VSLAM
     │                                      ├─► Localization
     │                                      └─► Mapping
     │
     └───────┬────────────┘
             ▼
    ┌────────────────┐
    │Arduino Interface│ ← ROS2 Topics
    │  (Serial JSON) │ → Serial Port
    └────────┬────────┘
             ▼
    ┌────────────────┐
    │ Arduino Mega   │
    │  (Firmware)    │
    ├────────────────┤
    │ • 6 IR Sensors │ → analogRead() every 20ms
    │ • 2 Motors     │ → PWM + Direction pins
    │ • Servo Gripper│ → Servo.write()
    │ • Buzzer       │ → digitalWrite()
    └────────────────┘
```

### **Key Interactions:**

1. **Mission Start:** User publishes to `/mission/start` → Mission Manager starts Zone A
2. **Zone A:** Arduino sends IR sensors → Zone A calculates PID → Commands motors
3. **Zone A Complete:** Zone A publishes 'complete' → Mission Manager starts Zone B
4. **Zone B Mapping:** RTAB-Map builds 3D map from RealSense RGB-D
5. **Color Detection:** ColorDetector finds 3 colored squares on East wall
6. **QR Detection:** QRDetector finds 3 QR codes on North wall
7. **Manipulation:** Zone B navigates, picks up coins, delivers to color squares
8. **Mission Complete:** Zone B publishes 'complete' → Mission Manager logs report

---

This is the complete breakdown of every component in the codebase! Each piece works together to create an autonomous mobile robot system.