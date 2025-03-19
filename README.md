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

This is a ROS2-based control framework for a bipedal robotic system with 8 degrees of freedom (DoF).  
Currently, it contains the basic structure, placeholders, framework illustration, and function prototypes that need to be verified in the future.

---

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

---

##  Getting Started

### Prerequisites

- **ROS2 Humble** (or later)
- **Ubuntu 22.04** (or later)
- **Python 3.10+**

---

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

---

## Run Instructions

Launch the entire system:

```bash
ros2 launch dodo_bringup dodo_robot.launch.py
```

---

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
| `dodo_bringup`      | Launch files and configuration    |
| `dodo_msgs`         | Custom message definitions        |

---

##  Development Environment Setup & Pitfalls

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

---

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


## License

Distributed under the MIT License. See `LICENSE` for more information.

