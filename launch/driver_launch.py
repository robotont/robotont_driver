from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    parameters_file_path = PathJoinSubstitution([
        FindPackageShare('robotont_driver'), 'config', 'parameters.yaml'
    ])

    return LaunchDescription([
        # Declare launch file arguments
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        # DeclareLaunchArgument('device_name', default_value='/dev/robotont'),
        # DeclareLaunchArgument('baud_rate', default_value='115200'),
        # DeclareLaunchArgument('flow_control', default_value='none'),
        # DeclareLaunchArgument('parity', default_value='none'),
        # DeclareLaunchArgument('stop_bits', default_value='one'),
        # DeclareLaunchArgument('plugin_odom', default_value='True'),
        # DeclareLaunchArgument('plugin_motor', default_value='True'),
        # DeclareLaunchArgument('plugin_led_module', default_value='True'),
        # DeclareLaunchArgument('plugin_power_supply', default_value='False'),
        # DeclareLaunchArgument('plugin_range', default_value='False'),

        Node(
            package='robotont_driver',
            executable='driver_node',
            name='driver',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                parameters_file_path,  # Base parameters from YAML
                {
                    # Override YAML with launch file arguments
                    'frame_prefix': LaunchConfiguration('frame_prefix'),
                    # 'device_name': LaunchConfiguration('device_name'),
                    # 'baud_rate': LaunchConfiguration('baud_rate'),
                    # 'flow_control': LaunchConfiguration('flow_control'),
                    # 'parity': LaunchConfiguration('parity'),
                    # 'stop_bits': LaunchConfiguration('stop_bits'),
                    # 'plugin_odom': LaunchConfiguration('plugin_odom'),
                    # 'plugin_motor': LaunchConfiguration('plugin_motor'),
                    # 'plugin_led_module': LaunchConfiguration('plugin_led_module'),
                    # 'plugin_power_supply': LaunchConfiguration('plugin_power_supply'),
                    # 'plugin_range': LaunchConfiguration('plugin_range'),
                }
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])

