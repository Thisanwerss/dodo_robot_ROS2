# Monitor Node

The Monitor Node is responsible for monitoring and logging the robot's state, performance, and diagnostic information. It collects data from various sources in the system and provides visualization, logging, and diagnostic capabilities.

## Features

- Monitors robot system state and health
- Logs sensor data, system events, and performance metrics
- Provides diagnostic information for troubleshooting
- Detects sensor failures or degradation
- Records data for offline analysis and debugging
- Configurable warning thresholds for different sensor types
- Supports both terminal output and diagnostic messaging

## Topics

### Subscribed Topics

- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData)
  - Synchronized sensor data from all sensors
  - Used for integrated system monitoring and diagnostics
  - Primary source of data for sensor performance analysis

### Published Topics

- `/sensor_diagnostics` (diagnostic_msgs/DiagnosticArray)
  - Detailed diagnostic information for each sensor
  - Contains status, warnings, and error messages
  - In dummy mode: Generates simulated diagnostic data for testing

## Parameters

- `imu_noise_threshold` (double, default: 0.05)
  - Maximum allowed IMU noise level in m/s²
  - Used to detect faulty or noisy IMU sensors

- `joint_position_noise_threshold` (double, default: 0.01)
  - Maximum allowed joint position noise in radians
  - Used to detect encoder issues or mechanical problems

- `check_rate` (int, default: 10)
  - Frequency (in Hz) at which monitoring checks are performed
  - Controls how often diagnostics are published

- `use_sim_time` (bool, default: false)
  - Whether to use simulation time
  - Used for synchronizing with simulation environments

- `dummy_mode` (bool, default: false)
  - When true, generates simulated monitoring data
  - Useful for testing monitoring systems without actual robot data

## Functions

### MonitorNode Class Functions

- `alignedSensorDataCallback`: Processes aligned sensor data
- `updateDiagnostics`: Updates the diagnostic information
- `publishDiagnostics`: Publishes diagnostic messages
- `checkIMUSensor`: Checks IMU sensor health
- `checkJointSensors`: Checks joint sensor health
- `detectAnomalies`: Detects anomalies in sensor data
- `calculateNoiseLevels`: Calculates noise levels in sensor readings

### Function Relationship

```
main()
└── MonitorNode
    ├── Constructor
    │   └── Get parameters
    ├── alignedSensorDataCallback() ← /aligned_sensor_data topic
    │   └── Store latest sensor data
    └── updateDiagnostics() [Timer callback]
        ├── checkIMUSensor()
        │   └── Evaluate IMU data quality
        ├── checkJointSensors()
        │   └── Evaluate joint sensor data quality
        ├── detectAnomalies()
        │   └── Identify unusual sensor patterns
        ├── calculateNoiseLevels()
        │   └── Measure sensor noise
        └── publishDiagnostics()
            └── Publish to /sensor_diagnostics topic
```