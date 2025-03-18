from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    usb_device_arg = DeclareLaunchArgument(
        'usb_device',
        default_value='/dev/input/js0',
        description='Path to the USB joystick device'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20',
        description='Rate at which commands are published in Hz'
    )

    # USB command node
    usb_command_node = Node(
        package='dodo_usb_command',
        executable='usb_command_node',
        name='usb_command_node',
        parameters=[{
            'usb_device': LaunchConfiguration('usb_device'),
            'publish_rate': LaunchConfiguration('publish_rate')
        }],
        output='screen'
    )

    return LaunchDescription([
        usb_device_arg,
        publish_rate_arg,
        usb_command_node
    ])