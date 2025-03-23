# Dodo Messages

The Dodo Messages package defines custom message types used throughout the Dodo robot system. These messages facilitate communication between different nodes and provide standardized data structures for robot state, sensor data, and control commands.

## Features

- Provides custom message definitions for the Dodo robot system
- Ensures consistent data structures across all nodes
- Includes time-synchronized sensor data messages
- Supports both real hardware and simulation environments
- Generated with proper ROS2 message dependencies

## Message Types

### AlignedSensorData

The primary custom message type that provides time-synchronized data from multiple sensors.

```
# Header with timestamp
std_msgs/Header header

# IMU data
sensor_msgs/Imu imu_data

# Joint states
sensor_msgs/JointState joint_states

# Status flags
bool imu_valid
bool joint_states_valid
```

**Purpose:**
- Provides a single message containing synchronized data from multiple sensors
- Ensures that all sensor readings correspond to the same point in time
- Includes validity flags to indicate sensor reliability
- Used by control, safety, and monitoring nodes for coordinated decision-making

## Usage

To use these messages in your ROS2 package:

1. Add a dependency in your `package.xml`:
   ```xml
   <depend>dodo_msgs</depend>
   ```

2. Add a dependency in your `CMakeLists.txt`:
   ```cmake
   find_package(dodo_msgs REQUIRED)
   ```

3. Include the message header in your C++ code:
   ```cpp
   #include <dodo_msgs/msg/aligned_sensor_data.hpp>
   ```

4. Subscribe to or publish the message:
   ```cpp
   // Example publisher
   auto publisher = create_publisher<dodo_msgs::msg::AlignedSensorData>("/aligned_sensor_data", 10);

   // Example subscriber
   auto subscription = create_subscription<dodo_msgs::msg::AlignedSensorData>(
     "/aligned_sensor_data", 10, std::bind(&YourClass::callback, this, std::placeholders::_1));
   ```

## Message Generation

The messages in this package are automatically generated during the build process using the ROS2 message generation system. The message definitions are located in the `msg` directory and are defined in `.msg` files.

Additional message types may be added to this package in the future as needed for enhanced functionality.