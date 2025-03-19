<p align="center">
    <img alt="MIRMI" src="./media/TUM_mirmi.png" height="80
    ">
</p>
# DoDo Alive! Project
DoDo Alive! is a course-based project at TUM MIRMI  
The Bipedal Robot Locomotion Task is a team comes from course students and motivated external contributers.  
Thanks for the support and supervision from:  
Dr. Hoan Quang Le  
Mr. Haowen Yao  

Team Members and Contributers:  
Dian Yu  
Shanshan Zhang  
Zherui Wang  
Denghui Yuan  
Jiahe Mao  


# Dodo Robot Bipedal Robot Locomotion ROS2 Framework

This is a ROS2-based control framework for a bipedal robotic system with 8 degrees of freedom (DoF).
Currently it only contains basic structure, place holders, framework illustration and functions prototype need to be verified in the future.

## Architecture

The framework consists of the following nodes:

- **RL Output Node**: Uses an offline-trained PPO model to compute joint position targets
- **USB Command Node**: Provides command input for front/back/left/right walking
- **Pre/Post Processing Node**: Handles data alignment and command processing
- **Odrive CANBUS Node**: Interfaces with motor drivers via CANBUS
- **IMU Sensor Node**: Provides raw IMU data
- **Sensor Fusion Node**: Fuses IMU and motor data
- **Safety Node**: Centralized safety management
- **Sensor Monitor Node**: Monitors sensor performance and diagnostics

## Getting Started

### Prerequisites

- ROS2 Humble or later
- Ubuntu 22.04 or later

### Building

```bash
cd dodo_robot_ws
colcon build
source install/setup.bash
```

### Running

Launch the entire system:

```bash
ros2 launch dodo_bringup dodo_robot.launch.py
```

## Packages

- `dodo_rl`: RL-based controller
- `dodo_usb_command`: USB command interface
- `dodo_processing`: Pre/post-processing of data and commands
- `dodo_canbus`: Motor control via CANBUS
- `dodo_imu`: IMU sensor interface
- `dodo_sensor_fusion`: Sensor fusion algorithms
- `dodo_safety`: Safety monitoring system
- `dodo_monitor`: Sensor monitoring
- `dodo_bringup`: Launch files and configuration
- `dodo_msgs`: Custom message definitions


