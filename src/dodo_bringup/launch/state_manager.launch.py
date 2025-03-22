from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
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

    # State Manager node
    state_manager_node = Node(
        package='dodo_bringup',
        executable='state_manager_node',
        name='state_manager_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        dummy_mode_arg,
        state_manager_node
    ])