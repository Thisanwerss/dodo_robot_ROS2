# Dodo IMU Node

This package provides an interface to the IMU sensor on the Dodo robot.

## Features

- Interfaces with IMU sensor via I2C
- Publishes raw IMU data
- Configurable sensor parameters

## Topics

### Published Topics
- `/imu_raw` (sensor_msgs/Imu): Raw IMU data

## Parameters

- `imu_device`: Path to the IMU device (default: "/dev/i2c-1")
- `imu_address`: I2C address of the IMU (default: 0x68)
- `publish_rate`: Rate at which IMU data is published (default: 100Hz)
- `frame_id`: Frame ID for IMU messages (default: "imu_link")