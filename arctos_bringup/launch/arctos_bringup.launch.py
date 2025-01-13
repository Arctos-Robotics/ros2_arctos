from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get package paths
    arctos_description_dir = get_package_share_directory('arctos_description')
    arctos_hardware_interface_dir = get_package_share_directory('arctos_hardware_interface')
    arctos_moveit_dir = get_package_share_directory('arctos_moveit_config')

    # Declare Launch Arguments
    declare_rviz_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=PathJoinSubstitution([arctos_description_dir, "config", "moveit.rviz"]),
        description="Path to RViz configuration file"
    )

    # RViz Configuration
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([arctos_description_dir, "urdf", "arctos.xacro"]),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Parameters
    motor_params = os.path.join(
        arctos_description_dir, 'config', 'arctos_controller.yaml'
    )

    # Nodes
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, motor_params],
        output={'stdout': 'screen', 'stderr': 'screen'},
        arguments=[
            '--ros-args',
            '--log-level', 'debug',
            '--log-level', 'arctos_hardware_interface:=debug',
            '--log-level', 'controller_manager:=debug',
        ],
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

    # Include CAN Launch
    can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arctos_hardware_interface_dir, "launch", "can_interface.launch.py"])
        )
    )

    # Include MoveIt Launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arctos_moveit_dir, "launch", "move_group.launch.py"])
        )
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Ensure joint state broadcaster starts before controllers
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_rviz_arg,
        LogInfo(msg=["Launching Arctos Bringup with RViz..."]),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        can_launch,
        move_group_launch,
        rviz_node,
    ])
