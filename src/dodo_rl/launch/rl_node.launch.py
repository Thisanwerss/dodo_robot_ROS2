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

    # RL node
    rl_node = Node(
        package='dodo_rl',
        executable='rl_node',
        name='rl_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'control_rate': LaunchConfiguration('control_rate'),
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']
        }],
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        control_rate_arg,
        rl_node
    ])