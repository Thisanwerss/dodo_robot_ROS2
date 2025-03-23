# CAN Bus Node

The CAN Bus Node serves as the interface between the robot's higher-level software and the motor controllers connected via the CAN bus interface. It translates high-level joint commands into motor-specific CAN messages and reports motor states back to the system.

## Features

- Provides a ROS interface to ODrive motor controllers connected via CAN bus
- Supports position control mode for multiple motors
- Handles emergency stop conditions
- Configurable PID gains for each motor
- Provides dummy mode for testing without hardware

## Topics

### Subscribed Topics

- `/processed_commands` (sensor_msgs/JointState)
  - Joint position commands processed by the Processing Node
  - Contains target positions for each motor
  - Used to send position commands to motors via CAN

- `/emergency_stop` (std_msgs/Bool)
  - Emergency stop signal
  - When `true`, immediately stops all motors and disables commands
  - High-priority safety feature

### Published Topics

- `/motor_states` (sensor_msgs/JointState)
  - Current motor states including position, velocity, and torque
  - In dummy mode: Simulated motor values that gradually move toward commanded positions
  - In real mode: Actual values read from motor controllers

## Parameters

- `can_interface` (string, default: "can0")
  - Name of the CAN interface to use
  - Determines which hardware CAN port connects to the motor network

- `update_rate` (int, default: 100)
  - Frequency (in Hz) at which the CAN bus is updated
  - Controls how often commands are sent and motor states are read

- `motor_ids` (integer array, default: [1, 2, 3, 4, 5, 6, 7, 8])
  - IDs of the motor controllers on the CAN bus
  - Used to address specific motors during command sending and state reading

- `pid_gains` (string, default: JSON string with PID values for each motor)
  - PID controller gains for each motor in JSON format
  - Contains kp (proportional), ki (integral), and kd (derivative) values
  - Example format: `{"1": {"kp": 10.0, "ki": 0.1, "kd": 0.01}, ...}`

- `use_sim_time` (bool, default: false)
  - Whether to use simulation time
  - Used for synchronizing with simulation environments

- `dummy_mode` (bool, default: false)
  - When true, simulates motor responses without actual hardware
  - Useful for testing and development without physical motors

## Functions

### CANBusNode Class Functions

- `processedCommandsCallback`: Processes incoming joint commands
- `emergencyStopCallback`: Handles emergency stop signals
- `updateCAN`: Main update function that sends commands and reads states
- `sendCommands`: Sends position commands to motors
- `readMotorStates`: Reads current states from motors
- `applyPIDGains`: Applies PID gains to the motor controllers
- `generateDummyMotorStates`: Generates simulated motor states for testing

### OdriveCANInterface Class Functions

- `init`: Initializes the CAN interface
- `close`: Closes the CAN interface
- `sendPositionCommand`: Sends a position command to a specific motor
- `readMotorStates`: Reads states from multiple motors
- `setPIDGains`: Sets PID gains for a specific motor
- `emergencyStop`: Triggers emergency stop on all motors

### Function Relationship

```
main()
└── CANBusNode
    ├── Constructor
    │   ├── Get parameters
    │   ├── Initialize OdriveCANInterface
    │   └── applyPIDGains()
    ├── processedCommandsCallback() ← /processed_commands topic
    ├── emergencyStopCallback() ← /emergency_stop topic
    ├── updateCAN() [Timer callback]
    │   ├── sendCommands()
    │   │   └── OdriveCANInterface::sendPositionCommand()
    │   └── readMotorStates()
    │       ├── OdriveCANInterface::readMotorStates() [Real mode]
    │       └── generateDummyMotorStates() [Dummy mode]
    └── Destructor
        └── OdriveCANInterface::close()
```