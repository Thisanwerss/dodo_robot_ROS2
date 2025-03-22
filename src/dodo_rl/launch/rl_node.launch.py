from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('dodo_rl')

    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_dir, 'models', 'ppo_model.pt'),
        description='Path to the pre-trained PPO model file'
    )

    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='100',
        description='Control loop rate in Hz'
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

    # RL node
    rl_node = Node(
        package='dodo_rl',
        executable='rl_node',
        name='rl_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'control_rate': LaunchConfiguration('control_rate'),
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }],
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        control_rate_arg,
        use_sim_time_arg,
        dummy_mode_arg,
        rl_node
    ])