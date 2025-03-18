# Dodo Monitor Node

This package implements sensor monitoring and diagnostics for the Dodo robot.

## Features

- Monitors sensor performance
- Publishes diagnostic information
- Detects sensor failures or degradation
- Provides configurable warning thresholds

## Topics

### Subscribed Topics
- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData): Time-aligned sensor data

### Published Topics
- `/sensor_diagnostics` (diagnostic_msgs/DiagnosticArray): Diagnostic information for sensors

## Parameters

- `imu_noise_threshold`: Maximum allowed IMU noise level (default: 0.05 m/s²)
- `joint_position_noise_threshold`: Maximum allowed joint position noise (default: 0.01 rad)
- `check_rate`: Rate at which monitoring checks are performed (default: 10Hz)