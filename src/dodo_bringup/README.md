# Dodo Bringup

The Dodo Bringup package is the central orchestration package for the Dodo robot system. It manages robot state transitions, system initialization, and coordinates the various nodes of the robot.

## Features

- Manages the robot's state machine
- Coordinates system startup and shutdown
- Handles state transitions between modes (STANDBY, MANUAL, AUTONOMOUS, etc.)
- Monitors system health and handles error conditions
- Provides launch files for the entire robot system
- Includes debugging tools for parameter validation

## Topics

### Subscribed Topics

- `/emergency_stop` (std_msgs/Bool)
  - Emergency stop signal
  - When `true`, transitions the robot to EMERGENCY_STOP state
  - High-priority safety feature

- `/usb_commands` (std_msgs/Int32)
  - Command signals from the USB controller
  - Used to transition from STANDBY to MANUAL control mode
  - Enables operator intervention

- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData)
  - Combined sensor data from multiple sensors with synchronized timestamps
  - Used to monitor system health and detect abnormal conditions
  - Triggers ERROR state transitions when appropriate

### Published Topics

- `/robot_state` (std_msgs/String)
  - Current state of the robot's state machine
  - Published states include: INACTIVE, INITIALIZING, STANDBY, MANUAL, AUTONOMOUS, ERROR, EMERGENCY_STOP
  - Allows other nodes to adapt their behavior based on robot state

## Parameters

- `use_sim_time` (bool, default: false)
  - Whether to use simulation time
  - Used for synchronizing with simulation environments

- `dummy_mode` (bool, default: false)
  - When true, the robot operates without hardware
  - Useful for testing and development without physical hardware

## Functions

### StateManagerNode Class Functions

- `publishState`: Publishes the current robot state
- `emergencyStopCallback`: Handles emergency stop signals
- `usbCommandsCallback`: Handles USB controller commands
- `alignedSensorDataCallback`: Processes sensor data for error detection
- `onStateChange`: Handles state transition events
- `initializeRobot`: Performs robot initialization sequence

### RobotStateMachine Class Functions

- `getCurrentState`: Returns the current robot state
- `getCurrentStateString`: Returns the current state as a string
- `processEvent`: Processes a state transition event
- `registerTransitionCallback`: Registers a callback for state transitions
- `stateToString`: Converts a state enum to string
- `eventToString`: Converts an event enum to string
- `initTransitionTable`: Initializes the state transition table
- `transitionTo`: Executes a state transition
- `isValidTransition`: Checks if a transition is valid

### ParameterDebuggingNode Class Functions

- `printParameters`: Prints and validates parameter values

### Function Relationship

```
main()
├── StateManagerNode
│   ├── Constructor
│   │   ├── Get parameters
│   │   └── initializeRobot()
│   ├── publishState() [Timer callback]
│   │   └── Publish to /robot_state topic
│   ├── emergencyStopCallback() ← /emergency_stop topic
│   │   └── state_machine_.processEvent()
│   ├── usbCommandsCallback() ← /usb_commands topic
│   │   └── state_machine_.processEvent()
│   ├── alignedSensorDataCallback() ← /aligned_sensor_data topic
│   │   └── state_machine_.processEvent()
│   └── onStateChange()
│       └── Execute actions based on new state
└── RobotStateMachine
    ├── processEvent()
    │   ├── isValidTransition()
    │   └── transitionTo()
    └── transitionTo()
        └── Execute transition callbacks
```

### Launch Files

The package includes several launch files:

- `dodo_robot.launch.py`: Main launch file that starts all robot nodes
- `hardware.launch.py`: Launches hardware-specific nodes
- `simulation.launch.py`: Launches simulation-specific nodes
- `state_manager.launch.py`: Launches the state manager node

## Usage

To launch the complete robot system:

```bash
ros2 launch dodo_bringup dodo_robot.launch.py
```

To launch in simulation mode:

```bash
ros2 launch dodo_bringup dodo_robot.launch.py use_sim_time:=true
```

To launch in dummy mode (without hardware):

```bash
ros2 launch dodo_bringup dodo_robot.launch.py dummy_mode:=true
```