#!/bin/bash

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source the workspace setup
source install/setup.bash

# Run the monitor GUI
ros2 launch dodo_monitor gui_node.launch.py