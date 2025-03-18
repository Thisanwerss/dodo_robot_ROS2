from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    max_time_diff_arg = DeclareLaunchArgument(
        'max_time_diff',
        default_value='0.01',
        description='Maximum time difference allowed between sensors in seconds'
    )

    fusion_rate_arg = DeclareLaunchArgument(
        'fusion_rate',
        default_value='100',
        description='Rate at which fused data is published in Hz'
    )

    # Fusion node
    fusion_node = Node(
        package='dodo_sensor_fusion',
        executable='fusion_node',
        name='fusion_node',
        parameters=[{
            'max_time_diff': LaunchConfiguration('max_time_diff'),
            'fusion_rate': LaunchConfiguration('fusion_rate')
        }],
        output='screen'
    )

    return LaunchDescription([
        max_time_diff_arg,
        fusion_rate_arg,
        fusion_node
    ])