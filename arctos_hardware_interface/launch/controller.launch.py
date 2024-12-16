from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Initialize the package directories
    pkg_dir = Path(get_package_share_directory("arctos_hardware_interface"))

    arctos_description_dir = Path(get_package_share_directory("arctos_description"))
    urdf_file = os.path.join(arctos_description_dir, 'urdf', 'arctos.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Launch Arguments
    model_name_arg = DeclareLaunchArgument(
        name="model_name",
        default_value="arctos",
        description="URDF model: Name of robot",
    )
    config_dir_arg = DeclareLaunchArgument(
        name="config_dir",
        default_value=str(pkg_dir / "config"),
        description="Directory which contains all controller configuration (.yaml) files",
    )
    controllers_arg = DeclareLaunchArgument(
        name="controllers",
        default_value="arctos_controller.yaml",
        description="Controllers config file, relative to the `config_dir` directory",
    )
    rviz_config_path_arg = DeclareLaunchArgument(
        name="rviz_config_path",
        default_value=str(pkg_dir / "rviz" / "view_robot.rviz"),
        description="Absolute path to rviz config file",
    )
    rviz_delay_arg = DeclareLaunchArgument(
        name="rviz_delay",
        default_value="3.0",
        description="Delay in seconds between joint state publisher and rviz launch to prevent error message",
    )

    # Robot State Publisher Node
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution([LaunchConfiguration("config_dir"), LaunchConfiguration("controllers")]),
        ],
        remappings=[
            ("/robot_description", "robot_description"),  # Ensure it listens to the correct topic
        ],
        output="both",
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # RViz Node with Delayed Launch
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_path")],
    )
    delay_rviz_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[TimerAction(
                period=LaunchConfiguration("rviz_delay"),
                actions=[rviz_node],
            )],
        )
    )

    # Controller Spawner
    arctos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_controller", "--controller-manager", "/controller_manager"],
    )
    delay_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arctos_controller_spawner],
        )
    )

    # Construct Launch Description
    return LaunchDescription([
        model_name_arg,
        config_dir_arg,
        controllers_arg,
        rviz_config_path_arg,
        rviz_delay_arg,
        robot_state_pub,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_rviz_node,
        delay_controller_spawners,
    ])
