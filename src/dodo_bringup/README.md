# Dodo Bringup Package

This package contains launch files and configuration for launching the complete Dodo robot system.

## Launch Files

- `dodo_robot.launch.py`: Main launch file that starts all the nodes for the Dodo robot
- `simulation.launch.py`: Launch file for running in simulation mode
- `hardware.launch.py`: Launch file for running on physical hardware

## Configuration

- `config/`: Configuration files for the robot nodes

## Usage

To launch the complete robot system:

```bash
ros2 launch dodo_bringup dodo_robot.launch.py
```

To launch in simulation mode:

```bash
ros2 launch dodo_bringup simulation.launch.py
```

To launch on physical hardware:

```bash
ros2 launch dodo_bringup hardware.launch.py
```