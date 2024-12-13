import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction

def generate_launch_description():
    arctos_description_dir = get_package_share_directory('arctos_description')
    arctos_hardware_interface_dir = get_package_share_directory('arctos_hardware_interface')
    arctos_moveit_dir = get_package_share_directory('arctos_moveit_config')

    # # Launch RViz

    # Launch MoveIt
    moveit_launch_file = os.path.join(arctos_moveit_dir, 'launch', 'move_group.launch.py')
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
    )

    can_launch_file = os.path.join(arctos_hardware_interface_dir, 'launch', 'can_interface.launch.py')
    can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(can_launch_file),
    )

    controller_launch_file= os.path.join(arctos_hardware_interface_dir, 'launch', 'arctos_controller.launch.py')
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file),
    )


    nodes = [
        moveit_launch,
        can_launch,
        controller_launch,
    ]

    return LaunchDescription(nodes)
