# AlignedSensorData.msg
# This message contains time-aligned data from multiple sensors

std_msgs/Header header

# IMU data
sensor_msgs/Imu imu_data

# Motor joint states
sensor_msgs/JointState joint_states

# Time alignment information
float64 time_offset  # Time offset between sensors in seconds