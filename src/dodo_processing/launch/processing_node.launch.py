from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
import json

def generate_launch_description():
    # Launch arguments
    command_rate_arg = DeclareLaunchArgument(
        'command_rate',
        default_value='100',
        description='Rate at which commands are processed and published in Hz'
    )

    motion_constraints_arg = DeclareLaunchArgument(
        'motion_constraints',
        default_value='''{
            "joint1": {"min_position": -1.57, "max_position": 1.57},
            "joint2": {"min_position": -1.57, "max_position": 1.57},
            "joint3": {"min_position": -1.57, "max_position": 1.57},
            "joint4": {"min_position": -1.57, "max_position": 1.57},
            "joint5": {"min_position": -1.57, "max_position": 1.57},
            "joint6": {"min_position": -1.57, "max_position": 1.57},
            "joint7": {"min_position": -1.57, "max_position": 1.57},
            "joint8": {"min_position": -1.57, "max_position": 1.57}
        }''',
        description='JSON string defining motion constraints for each joint'
    )

    # Processing node
    processing_node = Node(
        package='dodo_processing',
        executable='processing_node',
        name='processing_node',
        parameters=[{
            'command_rate': LaunchConfiguration('command_rate'),
            'motion_constraints': ParameterValue(LaunchConfiguration('motion_constraints'), value_type=str)
        }],
        output='screen'
    )

    return LaunchDescription([
        command_rate_arg,
        motion_constraints_arg,
        processing_node
    ])