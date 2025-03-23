<p align="center">
    <img alt="MIRMI" src="./media/TUM_mirmi.png" height="80">
</p>

# DoDo Alive! Project

DoDo Alive! is a course-based project at TUM MIRMI.  
The Bipedal Robot Locomotion Task is a team consisting of course students and motivated external contributors.  
Thanks for the support and supervision from:  
- Dr. Hoan Quang Le  
- Mr. Haowen Yao  

Team Members and Contributors:  
- Dian Yu  
- Shanshan Zhang  
- Zherui Wang  
- Denghui Yuan  
- Jiahe Mao  

---

# Dodo Robot Bipedal Robot Locomotion ROS2 Framework

This is a ROS2-based control framework for a bipedal robotic system with 8 degrees of freedom (DoF). The system integrates hardware interfaces, sensor processing, reinforcement learning, and safety systems to enable autonomous and manual control of the robot.

## System Overview

The Dodo robot is controlled by a series of ROS2 nodes that work together to provide autonomous and manual control capabilities. The system architecture follows a modular design where each component handles a specific aspect of the robot's operation.

### Key Components

- **State Management**: Central coordination of robot states (STANDBY, MANUAL, AUTONOMOUS, etc.)
- **Hardware Interface**: CAN-based motor control and IMU sensor interface
- **Sensor Fusion**: Time-synchronization of sensor data
- **Motion Processing**: Processes and validates motion commands
- **Reinforcement Learning**: Generates actions based on learned policies
- **Safety Systems**: Monitors robot operation for hazardous conditions
- **USB Control**: Interface for manual control via USB devices

## Architecture

The framework consists of the following nodes:

- **RL Node**: Uses an offline-trained PPO model to compute joint position targets  
- **USB Command Node**: Provides command input for front/back/left/right walking  
- **Processing Node**: Handles data alignment and command processing  
- **Odrive CANBUS Node**: Interfaces with motor drivers via CANBUS  
- **IMU Sensor Node**: Provides raw IMU data  
- **Sensor Fusion Node**: Fuses IMU and motor data  
- **Safety Node**: Centralized safety management  
- **Monitor Node**: Monitors sensor performance and diagnostics  
- **State Manager Node**: Manages robot state transitions

## System Architecture and Communication Flow

The Dodo robot's software architecture follows a modular design pattern where specialized nodes handle specific aspects of the robot's operation. The figure below illustrates the complete system architecture, including all nodes, topics, and message types.

<!-- Note: To generate the architecture diagram, install PlantUML and run:
     plantuml architecture.puml -o docs
     The diagram will show all nodes, topics, and message types with their relationships -->

```
                    ┌─────────────┐
                 ┌──┤  RL Node    │
                 │  └─────────────┘
                 │         │
                 │         ▼
                 │  ┌─────────────┐        ┌─────────────┐
┌─────────────┐  │  │ Processing  │        │   Safety    │
│     IMU     │──┼─►│    Node     │◄───────┤    Node     │
└─────────────┘  │  └─────────────┘        └─────────────┘
      │          │         │                      ▲
      ▼          │         ▼                      │
┌─────────────┐  │  ┌─────────────┐        ┌─────────────┐
│   Sensor    │──┼─►│  CAN Bus    │        │   Monitor   │
│   Fusion    │  │  │    Node     │◄───────┤    Node     │
└─────────────┘  │  └─────────────┘        └─────────────┘
      ▲          │         ▲                      ▲
      │          │         │                      │
┌─────────────┐  │  ┌─────────────┐               │
│   Motors    │──┘  │ USB Command │───────────────┘
└─────────────┘     │    Node     │
                    └─────────────┘
```

The system follows a layered approach:
- **Hardware Interface Layer**: Interacts directly with sensors and actuators
- **Data Processing Layer**: Handles sensor fusion and synchronization
- **Control Layer**: Generates motion commands through RL or manual input
- **System Management Layer**: Handles safety, monitoring, and state transitions

### Node Interactions

1. The **IMU Node** and **CAN Bus Node** interface with physical hardware and publish sensor data.
2. The **Sensor Fusion Node** combines this data into time-synchronized `AlignedSensorData` messages.
3. The **RL Node** uses this synchronized data to generate joint commands based on learned policies.
4. The **USB Command Node** provides manual control through a joystick interface.
5. The **Processing Node** combines, validates, and constraints commands before sending to motors.
6. The **Safety Node** continuously monitors sensors and can trigger emergency stops.
7. The **Monitor Node** provides diagnostics and system health information.
8. The **State Manager Node** coordinates the overall robot state and mode transitions.

### Key Message Types

- **sensor_msgs/JointState**: Used for motor commands and state
- **sensor_msgs/Imu**: Raw IMU data
- **dodo_msgs/AlignedSensorData**: Time-synchronized sensor information
- **std_msgs/Bool**: Used for emergency stop signals
- **std_msgs/String**: Used for robot state information
- **std_msgs/Int32**: Used for directional commands
- **diagnostic_msgs/DiagnosticArray**: System diagnostics and health information

**PlantUML source code for the architecture diagram is available in `architecture.puml`.**

## Getting Started

### Prerequisites

- **ROS2 Humble** (or later)
- **Ubuntu 22.04** (or later)
- **Python 3.10+**
- CAN interface (for hardware control)

## Build Instructions

```bash
# Clone the repo and enter workspace
cd ~/dodo_main/dodo_robot_ws

# Source ROS2 Humble setup
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source your overlay workspace after building
source install/setup.bash
```

## Run Instructions

### Complete System (with Hardware)

```bash
ros2 launch dodo_bringup dodo_robot.launch.py
```

### Simulation Mode

```bash
ros2 launch dodo_bringup dodo_robot.launch.py use_sim_time:=true
```

### Without Hardware (Dummy Mode)

```bash
ros2 launch dodo_bringup dodo_robot.launch.py dummy_mode:=true
```

## Packages

| Package Name        | Description                       |
|---------------------|-----------------------------------|
| `dodo_rl`           | RL-based controller               |
| `dodo_usb_command`  | USB command interface             |
| `dodo_processing`   | Pre/post-processing of data       |
| `dodo_canbus`       | Motor control via CANBUS          |
| `dodo_imu`          | IMU sensor interface              |
| `dodo_sensor_fusion`| Sensor fusion algorithms          |
| `dodo_safety`       | Safety monitoring system          |
| `dodo_monitor`      | Sensor monitoring diagnostics     |
| `dodo_bringup`      | Launch files and state management |
| `dodo_msgs`         | Custom message definitions        |

## Development Environment Setup & Pitfalls

### Python Dependencies
Ensure you're using the **system Python 3.10+**, not a Conda environment. ROS2 Humble expects system Python packages.

```bash
which python3
# Should output: /usr/bin/python3
```

Install required Python packages for ROS2 message generation:
```bash
sudo apt-get install python3-empy python3-setuptools python3-pip
```

You can check if `empy` is correctly installed:
```bash
python3 -c "import em; print(em.__file__)"
```

### Avoid Using Conda (Recommended)
Using Conda Python can cause issues with ROS2, especially during `colcon build`, because:
- ROS2 may invoke the wrong Python interpreter (e.g., `/home/ubuntu/miniconda3/bin/python3`)
- Packages like `rosidl_adapter` fail if they can't find `empy` in the expected Python environment.

If you **must** use Conda:
- Install `empy` and other ROS dependencies manually:
  ```bash
  pip install empy lark-parser
  ```
- Export the correct Python path manually (not recommended):
  ```bash
  export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/lib/python3.10/site-packages
  ```

## Common Issues & Fixes

### Problem: `ModuleNotFoundError: No module named 'em'`
- **Cause**: ROS2 needs `empy` but can't find it.
- **Fix**:
  ```bash
  sudo apt-get install python3-empy
  ```

### Problem: `colcon build` still calls the wrong Python (miniconda)
- **Cause**: ROS2 cached your old environment's Python path.
- **Fix**:
  ```bash
  rm -rf build/ install/ log/
  colcon build
  ```

### Problem: `colcon build` fails with rosidl_adapter errors
- **Cause**: Wrong Python interpreter or missing dependencies.
- **Fix**:
  - Ensure `which python3` returns `/usr/bin/python3`
  - Install missing ROS2 Python dependencies:
    ```bash
    sudo apt-get install python3-colcon-common-extensions
    sudo apt-get install python3-pybind11 python3-numpy
    ```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

Distributed under the MIT License. See `LICENSE` for more information.