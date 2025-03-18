from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    imu_device_arg = DeclareLaunchArgument(
        'imu_device',
        default_value='/dev/i2c-1',
        description='Path to the IMU device'
    )

    imu_address_arg = DeclareLaunchArgument(
        'imu_address',
        default_value='104',  # 0x68 in decimal
        description='I2C address of the IMU'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100',
        description='Rate at which IMU data is published in Hz'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages'
    )

    # IMU node
    imu_node = Node(
        package='dodo_imu',
        executable='imu_node',
        name='imu_node',
        parameters=[{
            'imu_device': LaunchConfiguration('imu_device'),
            'imu_address': LaunchConfiguration('imu_address'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'frame_id': LaunchConfiguration('frame_id')
        }],
        output='screen'
    )

    return LaunchDescription([
        imu_device_arg,
        imu_address_arg,
        publish_rate_arg,
        frame_id_arg,
        imu_node
    ])