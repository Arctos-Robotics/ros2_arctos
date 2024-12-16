# demo.launch.py
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arctos").to_moveit_configs()
    
    # Get the package directories
    pkg_moveit = get_package_share_directory('arctos_moveit_config')
    
    # Load controllers config
    controllers_config = load_yaml('arctos_moveit_config', 'config/moveit_controllers.yaml')
    ros2_controllers_config = load_yaml('arctos_moveit_config', 'config/ros2_controllers.yaml')
    
    # ROS2 Controller parameters
    controller_params = {'controller_manager': {'ros__parameters': {'update_rate': 100}}}
    if ros2_controllers_config:
        controller_params['controller_manager']['ros__parameters'].update(ros2_controllers_config)

    # Launch Nodes
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            controller_params,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            controllers_config or {},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Delay start of joint trajectory controller until joint state broadcaster is ready
    delay_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            robot_state_pub_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            delay_joint_trajectory_controller,
            move_group_node,
            rviz_node,
        ]
    )