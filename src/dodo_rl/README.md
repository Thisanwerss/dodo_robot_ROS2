# Dodo RL Node

This package implements a Reinforcement Learning-based controller for the Dodo robot using a pre-trained PPO (Proximal Policy Optimization) model.

## Features

- Publishes joint position targets at 100Hz
- Uses an offline-trained PPO model to compute actions
- 8 Degrees of Freedom (DoF) control

## Topics

### Published Topics
- `/rl_actions` (sensor_msgs/JointState): Joint position targets

## Parameters

- `model_path`: Path to the pre-trained PPO model file
- `control_rate`: Control loop rate (default: 100Hz)
- `joint_names`: List of joint names to control