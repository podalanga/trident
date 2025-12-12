#!/usr/bin/env zsh
# Installation Verification Script
# Checks if all dependencies and components are properly installed

set -e

echo "============================================"
echo "Trident Competition - Installation Checker"
echo "============================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# Check ROS2 installation
echo "Checking ROS2 Installation..."
if [ -z "$ROS_DISTRO" ]; then
    check_fail "ROS2 not sourced. Please run: source /opt/ros/humble/setup.bash"
    exit 1
else
    check_pass "ROS2 $ROS_DISTRO found"
fi

# Check Python version
echo ""
echo "Checking Python..."
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
if [ ! -z "$PYTHON_VERSION" ]; then
    check_pass "Python $PYTHON_VERSION"
else
    check_fail "Python 3 not found"
fi

# Check required Python packages
echo ""
echo "Checking Python Packages..."
python3 -c "import rclpy" 2>/dev/null && check_pass "rclpy" || check_fail "rclpy (install: sudo apt install ros-$ROS_DISTRO-rclpy)"
python3 -c "import cv2" 2>/dev/null && check_pass "opencv (cv2)" || check_fail "opencv (install: pip3 install opencv-contrib-python)"
python3 -c "import numpy" 2>/dev/null && check_pass "numpy" || check_fail "numpy (install: pip3 install \"numpy<2\")"
python3 -c "import serial" 2>/dev/null && check_pass "pyserial" || check_fail "pyserial (install: pip3 install pyserial)"
python3 -c "from pyzbar import pyzbar" 2>/dev/null && check_pass "pyzbar" || check_warn "pyzbar (optional, install: pip3 install pyzbar)"

# Check ROS2 packages
echo ""
echo "Checking ROS2 Packages..."
ros2 pkg list | grep -q "realsense2_camera" && check_pass "realsense2_camera" || check_warn "realsense2_camera (install: sudo apt install ros-$ROS_DISTRO-realsense2-camera)"
ros2 pkg list | grep -q "rtabmap_ros" && check_pass "rtabmap_ros" || check_warn "rtabmap_ros (install: sudo apt install ros-$ROS_DISTRO-rtabmap-ros)"
ros2 pkg list | grep -q "cv_bridge" && check_pass "cv_bridge" || check_fail "cv_bridge (install: sudo apt install ros-$ROS_DISTRO-cv-bridge)"

# Check workspace
echo ""
echo "Checking Workspace..."
if [ -f "src/trident_competition/package.xml" ]; then
    check_pass "Package structure found"
else
    check_fail "Package structure not found"
    exit 1
fi

# Check if built
if [ -d "install/trident_competition" ]; then
    check_pass "Workspace built"
else
    check_warn "Workspace not built yet. Run: ./build.sh"
fi

# Check Arduino connection
echo ""
echo "Checking Arduino Connection..."
ARDUINO_PORT=$(find /dev -name 'ttyACM*' -o -name 'ttyUSB*' 2>/dev/null | head -n 1)
if [ ! -z "$ARDUINO_PORT" ]; then
    check_pass "Arduino found at $ARDUINO_PORT"
    
    # Check serial permissions
    if groups $USER | grep -q dialout; then
        check_pass "User has serial port permissions"
    else
        check_warn "User not in dialout group. Run: sudo usermod -a -G dialout \$USER (then logout/login)"
    fi
else
    check_warn "No Arduino detected at /dev/ttyACM* or /dev/ttyUSB*"
fi

# Check RealSense
echo ""
echo "Checking RealSense Camera..."
if command -v rs-enumerate-devices &> /dev/null; then
    if rs-enumerate-devices 2>/dev/null | grep -q "Intel"; then
        check_pass "RealSense camera detected"
    else
        check_warn "RealSense SDK installed but no camera detected"
    fi
else
    check_warn "RealSense SDK not installed (optional for simulation)"
fi

# Summary
echo ""
echo "============================================"
echo "Installation Check Complete"
echo "============================================"
echo ""
echo "Next Steps:"
echo "1. Fix any failed (✗) checks above"
echo "2. Address warnings (⚠) if using real hardware"
echo "3. Build workspace: ./build.sh"
echo "4. Upload Arduino firmware (see arduino/README.md)"
echo "5. Run tests: see TEST_COMMANDS.md"
echo "6. Launch mission: ros2 launch trident_competition competition.launch.py"
echo ""
