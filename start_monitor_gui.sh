#!/bin/bash

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source the workspace setup
cd /home/ubuntu/dodo_main/dodo_robot_ws
source install/setup.bash

# Run the ROS2 tools
echo "Starting ROS2 monitoring tools..."

# Start several useful monitoring tools in parallel
rqt_graph &
rqt_topic &
rqt_console &

# Start our monitor node
ros2 run dodo_monitor monitor_node &

echo "Monitor tools started. Press Ctrl+C to exit."

# Wait for user to press Ctrl+C
trap "echo 'Shutting down...'; kill $(jobs -p); exit" SIGINT
while true; do
  sleep 1
done