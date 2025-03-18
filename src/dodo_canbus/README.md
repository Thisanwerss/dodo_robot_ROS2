# Dodo CANBUS Node

This package provides a CANBUS interface for controlling Odrive motor drivers on the Dodo robot.

## Features

- Interfaces with motor drivers via CANBUS
- Accepts PID parameters and target positions
- Publishes motor feedback and states
- Responds to emergency stop signals

## Topics

### Subscribed Topics
- `/processed_commands` (sensor_msgs/JointState): Processed commands for the motors
- `/emergency_stop` (std_msgs/Bool): Emergency stop signal

### Published Topics
- `/motor_states` (sensor_msgs/JointState): Feedback and states from motors

## Parameters

- `can_interface`: Name of the CAN interface (default: "can0")
- `update_rate`: Rate at which the CANBUS is updated (default: 100Hz)
- `motor_ids`: List of motor IDs on the CANBUS
- `pid_gains`: JSON string defining PID gains for each motor