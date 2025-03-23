# USB Command Node

The USB Command Node provides an interface for controlling the robot via USB-connected input devices such as controllers, keyboards, or joysticks. It translates user inputs into standardized command messages for robot control.

## Features

- Interfaces with USB input devices (controllers, keyboards, joysticks)
- Translates physical inputs into ROS command messages
- Provides manual control capabilities for the robot
- Configurable button mappings and sensitivity settings
- Supports hot-plugging of USB devices
- Includes dummy mode for testing without physical controllers

## Topics

### Subscribed Topics

- `/robot_state` (std_msgs/String)
  - Current state of the robot's state machine
  - Used to determine when USB commands should be accepted
  - Prevents control input during certain robot states

### Published Topics

- `/usb_commands` (std_msgs/Int32)
  - Integer command values representing different actions
  - Command values include:
    - 0: STOP
    - 1: FORWARD
    - 2: BACKWARD
    - 3: LEFT
    - 4: RIGHT
    - 5-9: Additional commands based on controller inputs
  - In dummy mode: Generates periodic test commands for system validation

## Parameters

- `usb_device` (string, default: "/dev/input/js0")
  - Path to the USB input device
  - Can be changed to support different controller types

- `publish_rate` (int, default: 20)
  - Frequency (in Hz) at which the controller is polled
  - Controls responsiveness of the control inputs

- `deadzone` (double, default: 0.1)
  - Deadzone threshold for analog inputs
  - Helps eliminate unintended inputs from small joystick movements

- `button_map` (string, default: JSON string with button mappings)
  - Maps physical buttons to command values
  - JSON format for customizable control schemes

- `use_sim_time` (bool, default: false)
  - Whether to use simulation time
  - Used for synchronizing with simulation environments

- `dummy_mode` (bool, default: false)
  - When true, generates simulated controller inputs
  - Useful for testing without physical USB controllers

## Functions

### USBCommandNode Class Functions

- `readUSBDevice`: Reads raw input from the USB device
- `processInputs`: Processes raw inputs into standardized commands
- `publishCommands`: Publishes command messages to ROS topics
- `robotStateCallback`: Processes robot state changes for control mode
- `configureDevice`: Configures the USB device and button mappings
- `generateDummyCommands`: Generates test commands in dummy mode

### Function Relationship

```
main()
└── USBCommandNode
    ├── Constructor
    │   ├── Get parameters
    │   └── configureDevice()
    ├── robotStateCallback() ← /robot_state topic
    │   └── Update control permissions
    └── processInputs() [Timer callback]
        ├── readUSBDevice() [Real mode]
        │   └── Read from physical controller
        ├── generateDummyCommands() [Dummy mode]
        │   └── Create simulated inputs
        └── publishCommands()
            └── Publish to /usb_commands topic
```