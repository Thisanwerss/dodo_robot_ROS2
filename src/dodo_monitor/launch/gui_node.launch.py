from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('dodo_monitor'), 'config')
    
    return LaunchDescription([
        Node(
            package='dodo_monitor',
            executable='gui_node',
            name='gui_node',
            output='screen',
            parameters=[{
                'imu_noise_threshold': 0.05,
                'joint_position_noise_threshold': 0.01,
                'check_rate': 10,
                'system_info_rate': 1,
            }],
        ),
    ])
