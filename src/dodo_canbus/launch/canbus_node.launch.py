from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ===== Launch Arguments =====
    can_interfaces_arg = DeclareLaunchArgument(
        'can_interfaces',
        default_value="['can0']",
        description='List of CAN interfaces to use (e.g., ["can0"])'
    )

    serial_ports_arg = DeclareLaunchArgument(
        'serial_ports',
        default_value="['/dev/ttyAM0']",
        description='List of serial ports to use (e.g., ["/dev/ttyUSB0"])'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='100',
        description='Control loop update rate (Hz)'
    )

    # 左腿 CAN 电机 ID
    motor_ids_can0_arg = DeclareLaunchArgument(
        'motor_ids_can0',
        default_value='[1, 2, 3, 4]',
        description='Motor IDs on CAN0 (left leg)'
    )

    # 右腿 串口 电机 ID
    motor_ids_serial0_arg = DeclareLaunchArgument(
        'motor_ids_serial0',
        default_value='[5, 6, 7, 8]',
        description='Motor IDs on Serial0 (right leg)'
    )

    # 电机类型（可选）
    motor_types_can0_arg = DeclareLaunchArgument(
        'motor_types_can0',
        default_value="['DM4310', 'DM4310', 'DM4310', 'DM4310']",
        description='Motor types for CAN0'
    )

    motor_types_serial0_arg = DeclareLaunchArgument(
        'motor_types_serial0',
        default_value="['DM4310', 'DM4310', 'DM4310', 'DM4310']",
        description='Motor types for Serial0'
    )



    # ===== 启动节点 =====
    multi_motor_node = Node(
        package='dodo_canbus',
        executable='canbus_node',  # 或 multi_motor_control_node
        name='multi_motor_control_node',
        output='screen',
        parameters=[{
            'can_interfaces': ParameterValue(LaunchConfiguration('can_interfaces'), value_type=str),
            'serial_ports': ParameterValue(LaunchConfiguration('serial_ports'), value_type=str),
            'motor_ids_can0': ParameterValue(LaunchConfiguration('motor_ids_can0'), value_type=str),
            'motor_ids_serial0': ParameterValue(LaunchConfiguration('motor_ids_serial0'), value_type=str),
            'motor_types_can0': ParameterValue(LaunchConfiguration('motor_types_can0'), value_type=str),
            'motor_types_serial0': ParameterValue(LaunchConfiguration('motor_types_serial0'), value_type=str),
            'update_rate': LaunchConfiguration('update_rate')
            
            
        }]
    )

    # ===== 返回 Launch Description =====
    return LaunchDescription([
        can_interfaces_arg,
        serial_ports_arg,
        motor_ids_can0_arg,
        motor_ids_serial0_arg,
        motor_types_can0_arg,
        motor_types_serial0_arg,
        update_rate_arg,
        multi_motor_node
    ])
