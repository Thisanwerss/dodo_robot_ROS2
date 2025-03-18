from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='12.0',
        description='Maximum allowed acceleration in m/s²'
    )

    max_tilt_arg = DeclareLaunchArgument(
        'max_tilt',
        default_value='0.5',
        description='Maximum allowed tilt angle in radians'
    )

    max_joint_velocity_arg = DeclareLaunchArgument(
        'max_joint_velocity',
        default_value='5.0',
        description='Maximum allowed joint velocity in rad/s'
    )

    check_rate_arg = DeclareLaunchArgument(
        'check_rate',
        default_value='50',
        description='Rate at which safety checks are performed in Hz'
    )

    # Safety node
    safety_node = Node(
        package='dodo_safety',
        executable='safety_node',
        name='safety_node',
        parameters=[{
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'max_tilt': LaunchConfiguration('max_tilt'),
            'max_joint_velocity': LaunchConfiguration('max_joint_velocity'),
            'check_rate': LaunchConfiguration('check_rate')
        }],
        output='screen'
    )

    return LaunchDescription([
        max_acceleration_arg,
        max_tilt_arg,
        max_joint_velocity_arg,
        check_rate_arg,
        safety_node
    ])