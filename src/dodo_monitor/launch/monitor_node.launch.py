from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    imu_noise_threshold_arg = DeclareLaunchArgument(
        'imu_noise_threshold',
        default_value='0.05',
        description='Maximum allowed IMU noise level in m/s²'
    )

    joint_position_noise_threshold_arg = DeclareLaunchArgument(
        'joint_position_noise_threshold',
        default_value='0.01',
        description='Maximum allowed joint position noise in rad'
    )

    check_rate_arg = DeclareLaunchArgument(
        'check_rate',
        default_value='10',
        description='Rate at which monitoring checks are performed in Hz'
    )

    # Monitor node
    monitor_node = Node(
        package='dodo_monitor',
        executable='monitor_node',
        name='monitor_node',
        parameters=[{
            'imu_noise_threshold': LaunchConfiguration('imu_noise_threshold'),
            'joint_position_noise_threshold': LaunchConfiguration('joint_position_noise_threshold'),
            'check_rate': LaunchConfiguration('check_rate')
        }],
        output='screen'
    )

    return LaunchDescription([
        imu_noise_threshold_arg,
        joint_position_noise_threshold_arg,
        check_rate_arg,
        monitor_node
    ])