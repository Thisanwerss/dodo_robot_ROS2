from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    dummy_mode_arg = DeclareLaunchArgument(
        'dummy_mode',
        default_value='false',
        description='Enable dummy mode for testing without hardware'
    )

    # Package paths
    rl_pkg_share = FindPackageShare('dodo_rl')
    usb_command_pkg_share = FindPackageShare('dodo_usb_command')
    processing_pkg_share = FindPackageShare('dodo_processing')
    canbus_pkg_share = FindPackageShare('dodo_canbus')
    imu_pkg_share = FindPackageShare('dodo_imu')
    sensor_fusion_pkg_share = FindPackageShare('dodo_sensor_fusion')
    safety_pkg_share = FindPackageShare('dodo_safety')
    monitor_pkg_share = FindPackageShare('dodo_monitor')

    # Include launch files for each node
    rl_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([rl_pkg_share, 'launch', 'rl_node.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    usb_command_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([usb_command_pkg_share, 'launch', 'usb_command.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    processing_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([processing_pkg_share, 'launch', 'processing_node.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    canbus_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([canbus_pkg_share, 'launch', 'canbus_node.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    imu_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([imu_pkg_share, 'launch', 'imu_node.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    fusion_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([sensor_fusion_pkg_share, 'launch', 'fusion_node.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    safety_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([safety_pkg_share, 'launch', 'safety_node.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    monitor_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([monitor_pkg_share, 'launch', 'monitor_node.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    # RQT Graph visualization
    rqt_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        arguments=[],
        output='screen'
    )

    # State Manager Node launch
    state_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('dodo_bringup'), 'launch', 'state_manager.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'dummy_mode': LaunchConfiguration('dummy_mode')
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        dummy_mode_arg,
        
        # Core nodes for robot control
        GroupAction([
            imu_node_launch,
            canbus_node_launch,
            fusion_node_launch,
            safety_node_launch,
            processing_node_launch,
            rl_node_launch,
            usb_command_node_launch,
            monitor_node_launch,
            state_manager_launch,
        ]),
        
        # Visualization
        # Uncomment to launch RQT graph at startup
        # rqt_node
    ])