from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotont_driver',
            namespace='robotont_driver',
            executable='fake_driver_node',
            name='driver',
            parameters=[],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robotont_description'), 'launch/display_simulated_robot.launch.py')
            ),
            launch_arguments={'rviz_fixed_frame': 'odom'}.items()
        )
    ])