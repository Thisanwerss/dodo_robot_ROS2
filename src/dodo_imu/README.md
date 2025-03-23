# IMU Node

The IMU Node interfaces with the robot's Inertial Measurement Unit (IMU) sensor, providing orientation, acceleration, and angular velocity data to the rest of the system. It supports both real hardware and a dummy mode for testing.

## Features

- Provides a ROS interface to the IMU sensor via I2C
- Publishes orientation, acceleration, and angular velocity data
- Offers driver abstraction for different IMU hardware
- Configurable update rates and filtering options
- Includes dummy mode for testing without hardware

## Topics

### Subscribed Topics

None. The IMU Node is a source node that only publishes data.

### Published Topics

- `/imu_data` (sensor_msgs/Imu)
  - Standard ROS IMU message containing:
    - Orientation as quaternion
    - Angular velocity (x, y, z)
    - Linear acceleration (x, y, z)
    - Covariance matrices for each measurement
  - In dummy mode: Generates simulated IMU data with realistic noise patterns
  - In real mode: Actual measurements from the physical IMU sensor

## Parameters

- `imu_device` (string, default: "/dev/i2c-1")
  - Path to the IMU device
  - Determines which hardware device is used for IMU readings

- `imu_address` (int, default: 0x68)
  - I2C address of the IMU sensor
  - Used to communicate with the correct device on the I2C bus

- `publish_rate` (int, default: 100)
  - Frequency (in Hz) at which IMU data is published
  - Higher rates provide more responsive motion detection but use more bandwidth

- `frame_id` (string, default: "imu_link")
  - Frame ID for the IMU measurements in the TF tree
  - Used for coordinate transformations and sensor fusion

- `use_sim_time` (bool, default: false)
  - Whether to use simulation time
  - Used for synchronizing with simulation environments

- `dummy_mode` (bool, default: false)
  - When true, generates simulated IMU data without using hardware
  - Useful for testing and development without a physical IMU

## Functions

### IMUNode Class Functions

- `readIMU`: Reads data from the IMU sensor
- `publishIMU`: Publishes IMU data to the ROS topic
- `configureSensor`: Configures the IMU sensor parameters
- `calibrateSensor`: Performs calibration of the IMU sensor
- `generateDummyIMUData`: Generates simulated IMU data for testing

### IMUDriver Class Functions

- `connect`: Establishes connection to the IMU device
- `disconnect`: Closes the connection to the IMU device
- `readData`: Reads raw data from the IMU sensor
- `configureDevice`: Sets up device-specific configurations
- `performCalibration`: Executes the calibration procedure
- `applyFilter`: Applies filtering to raw sensor readings

### Function Relationship

```
main()
└── IMUNode
    ├── Constructor
    │   ├── Get parameters
    │   ├── Initialize IMUDriver
    │   └── configureSensor()
    ├── readIMU() [Timer callback]
    │   ├── IMUDriver::readData() [Real mode]
    │   │   └── IMUDriver::applyFilter()
    │   └── generateDummyIMUData() [Dummy mode]
    ├── publishIMU()
    │   └── Publish to /imu_data topic
    └── Destructor
        └── IMUDriver::disconnect()
```