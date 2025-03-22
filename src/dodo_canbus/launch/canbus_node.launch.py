from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='Name of the CAN interface'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='100',
        description='Rate at which the CANBUS is updated in Hz'
    )

    pid_gains_arg = DeclareLaunchArgument(
        'pid_gains',
        default_value='''{
            "1": {"kp": 10.0, "ki": 0.1, "kd": 0.01},
            "2": {"kp": 10.0, "ki": 0.1, "kd": 0.01},
            "3": {"kp": 10.0, "ki": 0.1, "kd": 0.01},
            "4": {"kp": 10.0, "ki": 0.1, "kd": 0.01},
            "5": {"kp": 10.0, "ki": 0.1, "kd": 0.01},
            "6": {"kp": 10.0, "ki": 0.1, "kd": 0.01},
            "7": {"kp": 10.0, "ki": 0.1, "kd": 0.01},
            "8": {"kp": 10.0, "ki": 0.1, "kd": 0.01}
        }''',
        description='JSON string defining PID gains for each motor'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    dummy_mode_arg = DeclareLaunchArgument(
        'dummy_mode',
        default_value='false',
        description='Use dummy mode for testing without hardware'
    )

    # CANBUS node
    canbus_node = Node(
        package='dodo_canbus',
        executable='canbus_node',
        name='canbus_node',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'update_rate': LaunchConfiguration('update_rate'),
            'motor_ids': [1, 2, 3, 4, 5, 6, 7, 8],
            'pid_gains': LaunchConfiguration('pid_gains'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }],
        output='screen'
    )

    return LaunchDescription([
        can_interface_arg,
        update_rate_arg,
        pid_gains_arg,
        use_sim_time_arg,
        dummy_mode_arg,
        canbus_node
    ])