from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # Get package paths
    arctos_hardware_interface_dir = get_package_share_directory('arctos_hardware_interface')
    arctos_moveit_dir = get_package_share_directory('arctos_moveit_config')

    # MoveItConfigsBuilder automatically do the following:
    # .robot_description: create urdf file using command: xacro arctos.urdf.xacro
    # .robot_description_semantic
    moveit_config = (
        MoveItConfigsBuilder("arctos")
        .robot_description(file_path="config/arctos.urdf.xacro")
        .robot_description_semantic(file_path="config/arctos.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .to_moveit_configs()
    )

    rviz_base = os.path.join(get_package_share_directory("arctos_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit_chomp.rviz")
    rviz_empty_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_empty_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    # Nodes
    # publish the state of robot to TF (transform)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Parameters
    robot_controllers = os.path.join(
        arctos_moveit_dir, 'config', 'ros2_controllers.yaml'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output={'stdout': 'screen', 'stderr': 'screen'},
        arguments=[
            '--ros-args',
            # '--log-level', 'debug',
            '--log-level', 'arctos_hardware_interface:=info',
            '--log-level', 'controller_manager:=info'
        ],
        remappings={
             ("/controller_manager/robot_description", "/robot_description"),
        }
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["denso_arm_controller", "--controller-manager", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arctos_hand_controller", "--controller-manager", "/controller_manager"],
    )

    # # Include CAN Launch
    # can_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([arctos_hardware_interface_dir, "launch", "can_interface.launch.py"])
    #     )
    # )

    # Include MoveIt Launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([arctos_moveit_dir, "launch", "move_group.launch.py"])
        )
    )

    # Ensure joint state broadcaster starts before controllers
    delay_robot_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_arm_controller_spawner, robot_hand_controller_spawner],
        )
    )

    # Delay rviz and moveit launch until controllers are ready
    delay_rviz_and_moveit_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_arm_controller_spawner,
            on_exit=[rviz_node, move_group_launch]
        ))
    
    return LaunchDescription([
        LogInfo(msg=["Launching Arctos Bringup with RViz..."]),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_arm_controller_spawner,
        delay_rviz_and_moveit_launch,
        # can_launch
    ])
