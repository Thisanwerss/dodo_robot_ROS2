# Reinforcement Learning Node

The Reinforcement Learning (RL) Node is responsible for running trained reinforcement learning models that control the robot's movements. It generates joint position targets based on sensor data and the robot's current state.

## Features

- Runs pretrained PPO (Proximal Policy Optimization) models
- Generates optimal joint positions based on current sensor readings
- Supports different policies for various tasks (walking, standing, etc.)
- Implements model inference with configurable update rates
- Provides both autonomous control and manual override capabilities
- Includes dummy mode for testing without a trained model

## Topics

### Subscribed Topics

- `/aligned_sensor_data` (dodo_msgs/AlignedSensorData)
  - Time-aligned sensor data from multiple sources
  - Used as the observation input to the RL model
  - Contains the robot state information needed for decision making

- `/robot_state` (std_msgs/String)
  - Current state of the robot's state machine
  - Used to determine when to enable/disable RL control
  - Controls whether RL actions are published or suppressed

### Published Topics

- `/rl_actions` (sensor_msgs/JointState)
  - Joint positions, velocities, and torques generated by the RL model
  - Primary output that drives the robot's motion
  - In dummy mode: Generates simple oscillatory patterns for testing
  - Used by the Processing Node to generate actual motor commands

## Parameters

- `model_path` (string, default: "")
  - Path to the trained RL model file
  - Can be a full model or just the policy weights

- `control_rate` (int, default: 100)
  - Frequency (in Hz) at which the RL model is updated and actions are published
  - Controls how often new joint positions are generated

- `joint_names` (string array)
  - List of joint names to control
  - Must match the joint names used in the rest of the system

- `observation_dim` (int, default: 32)
  - Dimension of the observation vector
  - Must match the input dimension of the trained model

- `action_dim` (int, default: 8)
  - Dimension of the action vector
  - Typically equal to the number of controllable joints

- `use_sim_time` (bool, default: false)
  - Whether to use simulation time
  - Used for synchronizing with simulation environments

- `dummy_mode` (bool, default: false)
  - When true, generates simulated actions without using the actual model
  - Useful for testing the action pipeline without a trained model

## Functions

### RLNode Class Functions

- `alignedSensorDataCallback`: Processes incoming sensor data
- `robotStateCallback`: Processes robot state changes
- `updateModel`: Updates the RL model with new observations
- `publishActions`: Publishes the actions generated by the model
- `prepareObservation`: Converts sensor data to model input format
- `generateDummyActions`: Generates dummy actions for testing

### PPOModel Class Functions

- `loadModel`: Loads a trained model from file
- `predict`: Generates actions based on observations
- `preprocessObservation`: Prepares observations for the model
- `postprocessAction`: Converts raw model outputs to actuator commands

### Function Relationship

```
main()
└── RLNode
    ├── Constructor
    │   ├── Get parameters
    │   └── Initialize PPOModel
    ├── alignedSensorDataCallback() ← /aligned_sensor_data topic
    │   └── Store latest sensor data
    ├── robotStateCallback() ← /robot_state topic
    │   └── Update control mode
    └── updateModel() [Timer callback]
        ├── prepareObservation()
        │   └── Format sensor data for model input
        ├── In autonomous mode:
        │   ├── PPOModel::predict() [Real mode]
        │   │   ├── PPOModel::preprocessObservation()
        │   │   └── PPOModel::postprocessAction()
        │   └── generateDummyActions() [Dummy mode]
        └── publishActions()
            └── Publish to /rl_actions topic
```