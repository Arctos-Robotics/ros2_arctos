from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the configuration YAML
    config_file = os.path.join(
        get_package_share_directory('arctos_hardware_interface'),
        'config',
        'arctos_hardware_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file],
            output='screen'
        ),
    ])
