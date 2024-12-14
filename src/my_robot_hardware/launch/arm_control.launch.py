# launch/arm_control.launch.py
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_hardware')
    
    # Get URDF via xacro
    urdf_file = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Load the controllers config file
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')

    # CAN interface nodes
    socketcan_receiver = Node(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socketcan_receiver_node',
        parameters=[{
            'interface': 'can0',
        }]
    )

    socketcan_sender = Node(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socketcan_sender_node',
        parameters=[{
            'interface': 'can0',
        }]
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Joint trajectory controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay joint trajectory controller until joint state broadcaster is ready
    delay_joint_trajectory_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription([
        socketcan_receiver,
        socketcan_sender,
        robot_state_pub,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_joint_trajectory_spawner
    ])