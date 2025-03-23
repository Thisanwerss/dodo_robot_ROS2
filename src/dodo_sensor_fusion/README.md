# Sensor Fusion Node

The Sensor Fusion Node integrates data from various sensors on the robot, including IMU, joint states, and other sources to provide a synchronized and aligned sensor data stream for the robot's control and monitoring systems.

## Features

- Synchronizes data from multiple sensor sources
- Aligns different sensor readings by timestamp
- Performs filtering and outlier rejection
- Generates fused and time-aligned sensor data
- Configurable fusion rates and methods
- Supports data recording and playback for debugging

## Topics

### Subscribed Topics

- `/imu_raw` (sensor_msgs/Imu)
  - Raw IMU data from the IMU sensor
  - Contains orientation, angular velocity, and linear acceleration

- `/motor_states` (sensor_msgs/JointState)
  - Current motor positions, velocities, and torques
  - Used for state estimation and sensor fusion

### Published Topics

- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData)
  - Custom message containing time-aligned readings from all sensors
  - Includes synchronized IMU data, joint states, and derived data
  - In dummy mode: Generates simulated sensor data with consistent timestamps
  - Used by higher-level control, monitoring, and state estimation nodes

## Parameters

- `fusion_rate` (int, default: 100)
  - Frequency (in Hz) at which sensor data is fused and published
  - Controls the update rate of the aligned sensor data

- `buffer_size` (int, default: 10)
  - Size of the internal buffer for sensor data
  - Larger buffers provide better synchronization but introduce more latency

- `max_time_diff` (double, default: 0.01)
  - Maximum allowed time difference between sensor readings to be considered synchronized
  - In seconds, lower values enforce stricter synchronization

- `use_sim_time` (bool, default: false)
  - Whether to use simulation time
  - Used for synchronizing with simulation environments

- `dummy_mode` (bool, default: false)
  - When true, generates simulated sensor data without relying on actual sensor inputs
  - Useful for testing and development without physical hardware

## Functions

### FusionNode Class Functions

- `imuCallback`: Processes incoming IMU data
- `motorStatesCallback`: Processes incoming motor state data
- `fuseData`: Main function that fuses data from different sensors
- `synchronizeData`: Synchronizes sensor readings by timestamp
- `publishFusedData`: Publishes aligned sensor data
- `generateDummyData`: Generates simulated sensor data for testing

### Function Relationship

```
main()
└── FusionNode
    ├── Constructor
    │   └── Get parameters
    ├── imuCallback() ← /imu_raw topic
    │   └── Store IMU data in buffer
    ├── motorStatesCallback() ← /motor_states topic
    │   └── Store motor states in buffer
    └── fuseData() [Timer callback]
        ├── synchronizeData() [Real mode]
        │   └── Find time-aligned data points
        ├── generateDummyData() [Dummy mode]
        │   └── Create synthetic aligned data
        └── publishFusedData()
            └── Publish to /aligned_sensor_data topic
```