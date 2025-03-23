# Safety Node

The Safety Node is responsible for monitoring robot operation and ensuring safe operation by detecting hazardous conditions and triggering emergency stops when necessary. It acts as a watchdog for the robot system.

## Features

- Monitors sensor data for unsafe conditions
- Triggers emergency stops when hazards are detected
- Monitors IMU data for anomalies (excessive accelerations, tilts)
- Monitors joint positions and velocities for anomalies
- Logs safety events for diagnostics and debugging
- Provides configurable safety parameters and thresholds
- Operates independently from control systems as a safety layer

## Topics

### Subscribed Topics

- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData)
  - Time-aligned sensor data from multiple sensors
  - Used to detect unsafe conditions based on sensor readings
  - Primary source of data for safety monitoring

### Published Topics

- `/emergency_stop` (std_msgs/Bool)
  - Emergency stop signal
  - When `true`, commands all systems to enter a safe state
  - In dummy mode: Generates test emergency signals for system validation

## Parameters

- `max_acceleration` (double, default: 12.0)
  - Maximum allowed acceleration in m/s²
  - Triggers emergency stop if exceeded

- `max_tilt` (double, default: 0.5)
  - Maximum allowed tilt angle in radians (≈ 28.6°)
  - Detects when robot is tipping over

- `max_joint_velocity` (double, default: 5.0)
  - Maximum allowed joint velocity in rad/s
  - Prevents dangerous joint movements

- `check_rate` (int, default: 50)
  - Frequency (in Hz) at which safety checks are performed
  - Higher rates provide faster response to dangerous conditions

- `use_sim_time` (bool, default: false)
  - Whether to use simulation time
  - Used for synchronizing with simulation environments

- `dummy_mode` (bool, default: false)
  - When true, runs in simulation mode without triggering hardware emergency stops
  - Useful for testing safety features without affecting physical systems

## Functions

### SafetyNode Class Functions

- `alignedSensorDataCallback`: Processes aligned sensor data
- `checkSafety`: Main safety checking function
- `checkAcceleration`: Checks for unsafe acceleration
- `checkTilt`: Checks for excessive tilt angles
- `checkJointVelocities`: Checks for unsafe joint velocities
- `triggerEmergencyStop`: Triggers an emergency stop
- `logSafetyEvent`: Logs safety-related events
- `publishSafetyStatus`: Publishes safety diagnostic information

### Function Relationship

```
main()
└── SafetyNode
    ├── Constructor
    │   └── Get parameters
    ├── alignedSensorDataCallback() ← /aligned_sensor_data topic
    │   └── Store latest sensor data
    └── checkSafety() [Timer callback]
        ├── checkAcceleration()
        │   └── Check for excessive acceleration
        ├── checkTilt()
        │   └── Check for excessive tilt angles
        ├── checkJointVelocities()
        │   └── Check for excessive joint speeds
        ├── triggerEmergencyStop() [if unsafe condition detected]
        │   └── Publish to /emergency_stop topic
        └── logSafetyEvent()
            └── Log details about safety events
```