# Dodo Safety Node

This package implements a centralized safety management system for the Dodo robot. It monitors aligned sensor data, detects falls/noisy readings, and triggers emergency stop with logging.

## Features

- Monitors IMU data for anomalies (excessive accelerations, tilts)
- Monitors joint positions and velocities for anomalies
- Triggers emergency stop when anomalies are detected
- Logs safety events
- Provides configurable safety thresholds

## Topics

### Subscribed Topics
- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData): Time-aligned sensor data

### Published Topics
- `/emergency_stop` (std_msgs/Bool): Emergency stop signal

## Parameters

- `max_acceleration`: Maximum allowed acceleration (default: 12.0 m/s²)
- `max_tilt`: Maximum allowed tilt angle (default: 0.5 rad ≈ 28.6°)
- `max_joint_velocity`: Maximum allowed joint velocity (default: 5.0 rad/s)
- `check_rate`: Rate at which safety checks are performed (default: 50Hz)