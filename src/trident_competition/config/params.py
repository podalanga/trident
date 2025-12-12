"""Configuration parameters for the competition"""

# Zone A - Line Following PID
LINE_FOLLOW_PID = {
    'kp': 0.8,
    'ki': 0.01,
    'kd': 0.3
}

BASE_SPEED = 150  # PWM 0-255
MAX_SPEED = 200
UPDATE_RATE = 50.0  # Hz

# Zone B - VSLAM and Navigation
VSLAM_UPDATE_RATE = 30  # Hz
ROTATION_SPEED = 0.1  # rad/s for 360° mapping

# Wall standoff distances (meters)
WALL_STANDOFF_PICKUP = 0.2  # 200mm
WALL_STANDOFF_DROP = 0.1    # 100mm

# Navigation tolerances
NAVIGATION_TOLERANCE = 0.05      # meters (50mm)
ORIENTATION_TOLERANCE = 0.087    # radians (~5 degrees)

# Vision - Color Detection
COLOR_DETECTION_ATTEMPTS = 5
MORPH_KERNEL_SIZE = 5  # pixels
MIN_SQUARE_AREA = 8000  # pixels (approximate at typical distance)
MAX_SQUARE_AREA = 12000

# Vision - QR Detection
QR_DETECTION_ATTEMPTS = 5
QR_MIN_DISTANCE = 0.3  # meters (300mm)
QR_MAX_DISTANCE = 0.5  # meters (500mm)

# Gripper
GRIPPER_CLOSE_ANGLE = 30  # degrees
GRIPPER_OPEN_ANGLE = 90
GRIPPER_ACTION_DELAY = 1.5  # seconds

# Timing
BUZZER_BEEP_DURATION = 0.3  # seconds
BUZZER_PAUSE_DURATION = 0.5

# Safety
MAX_MISSION_TIME = 600  # seconds (10 minutes)
MIN_BATTERY_VOLTAGE = 10.5  # volts
MIN_WALL_CLEARANCE = 0.05  # meters (50mm)

# Serial Communication
ARDUINO_SERIAL_PORT = '/dev/ttyACM0'
ARDUINO_BAUD_RATE = 115200
SERIAL_TIMEOUT = 1.0

# Camera
CAMERA_TOPIC_RGB = '/camera/color/image_raw'
CAMERA_TOPIC_DEPTH = '/camera/depth/image_rect_raw'
CAMERA_TOPIC_INFO = '/camera/color/camera_info'

# VSLAM
VSLAM_POSE_TOPIC = '/rtabmap/localization_pose'
VSLAM_MAP_TOPIC = '/rtabmap/map'
