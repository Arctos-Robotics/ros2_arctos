from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the description package path
    description_pkg_dir = get_package_share_directory('arctos_description')
    
    # Load hardware configuration
    motor_config = os.path.join(
        description_pkg_dir,
        'config',
        'hardware',
        'motor_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='arctos_hardware_interface',
            executable='arctos_hardware_interface_node',
            name='arctos_hardware_interface',
            parameters=[motor_config],
            output='screen'
        )
    ])