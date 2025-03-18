# Dodo Sensor Fusion Node

This package implements sensor fusion algorithms that combine IMU and motor data for the Dodo robot.

## Features

- Fuses IMU and motor encoder data
- Time synchronization of sensor data
- Provides aligned sensor data for other nodes

## Topics

### Subscribed Topics
- `/imu_raw` (sensor_msgs/Imu): Raw IMU data
- `/motor_states` (sensor_msgs/JointState): Feedback from motors

### Published Topics
- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData): Time-aligned sensor data

## Parameters

- `max_time_diff`: Maximum time difference allowed between sensors (default: 0.01s)
- `fusion_rate`: Rate at which fused data is published (default: 100Hz)