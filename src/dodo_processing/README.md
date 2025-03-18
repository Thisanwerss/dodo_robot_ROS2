# Dodo Processing Node

This package implements pre- and post-processing for data alignment and command processing for the Dodo robot.

## Features

- Processes RL actions into motor commands
- Processes USB commands into robot movements
- Processes sensor data for alignment
- Applies motion filters and constraints

## Topics

### Subscribed Topics
- `/rl_actions` (sensor_msgs/JointState): Joint position targets from RL
- `/usb_commands` (std_msgs/Int32): Command input for walking direction
- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData): Aligned sensor data

### Published Topics
- `/processed_commands` (sensor_msgs/JointState): Processed commands for the motors

## Parameters

- `command_rate`: Rate at which commands are processed and published (default: 100Hz)
- `motion_constraints`: JSON string defining motion constraints for each joint