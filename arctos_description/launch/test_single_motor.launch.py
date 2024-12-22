from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package paths
    pkg_description = FindPackageShare('arctos_description')
    arctos_hardware_interface_dir = get_package_share_directory('arctos_hardware_interface')
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_description, "urdf", "test_single_motor.urdf.xacro"]),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Get parameters file
    motor_params = os.path.join(
        get_package_share_directory('arctos_description'),
        'config',
        'hardware',
        'test_motor_config.yaml'
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, motor_params],
        output={'stdout': 'screen', 'stderr': 'screen'},
        arguments=['--ros-args', '--log-level', 'debug']
    )

    can_launch_file = os.path.join(arctos_hardware_interface_dir, 'launch', 'can_interface.launch.py')
    can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(can_launch_file),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        can_launch
    ]

    return LaunchDescription(nodes)