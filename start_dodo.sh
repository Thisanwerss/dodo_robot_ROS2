#!/bin/bash

# Exit on error
set -e

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}===== Starting Dodo Robot System in Dummy Mode =====${NC}"

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source ROS 2 setup
echo -e "${GREEN}Sourcing ROS 2 setup.bash...${NC}"
source /opt/ros/humble/setup.bash

# Clean the build artifacts if there are issues
if [ "$1" == "--clean" ]; then
    echo -e "${GREEN}Cleaning build directory...${NC}"
    rm -rf build/
    rm -rf install/
    rm -rf log/
fi

# Build the workspace
echo -e "${GREEN}Building the workspace...${NC}"
colcon build --symlink-install || {
    echo -e "${RED}Build failed! Try running with --clean option: ./start_dodo.sh --clean${NC}"
    exit 1
}

# Source the workspace after build
echo -e "${GREEN}Sourcing workspace setup.bash after build...${NC}"
source "$SCRIPT_DIR/install/setup.bash"

# Launch the robot in dummy mode
echo -e "${GREEN}Launching Dodo Robot in dummy mode...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop the robot${NC}"
ros2 launch --debug dodo_bringup dodo_robot.launch.py dummy_mode:=true --show-all

# This line will only be reached if ros2 launch exits normally
echo -e "${GREEN}Robot system has been shut down.${NC}"
