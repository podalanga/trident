#!/usr/bin/env zsh
# Build and source the workspace

set -e

SCRIPT_DIR="${0:a:h}"
WORKSPACE_DIR="$SCRIPT_DIR"

echo "Building Trident Competition workspace..."
cd "$WORKSPACE_DIR"

# Clean previous build (optional)
# rm -rf build/ install/ log/

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
source install/setup.bash

echo ""
echo "✓ Build complete!"
echo ""
echo "To use the workspace, run:"
echo "  source install/setup.bash"
echo ""
echo "To launch the competition:"
echo "  ros2 launch trident_competition competition.launch.py"
