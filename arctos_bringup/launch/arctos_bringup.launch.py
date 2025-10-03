import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Directories
    description_dir = get_package_share_directory('arctos_description')
    bringup_dir = get_package_share_directory('arctos_bringup')

    # Robot description from Xacro
    urdf_file = os.path.join(description_dir, 'urdf', 'arctos.xacro')
    srdf_file = os.path.join(description_dir, 'config', 'arctos.srdf')

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        TextSubstitution(text=' '),
        urdf_file
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Correct way to read SRDF
    with open(srdf_file, 'r') as f:
        srdf_content = f.read()


    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # Kinematics config
    robot_description_kinematics = os.path.join(description_dir, 'config', 'kinematics.yaml')

    # MoveIt planners config
    moveit_planners = os.path.join(description_dir, 'config', 'moveit_planners.yaml')

    # ROS2 Control config
    hardware_config = os.path.join(description_dir, 'config', 'ros2_controllers.yaml')

    # MoveIt controller config
    moveit_controllers = os.path.join(description_dir, 'config', 'moveit_controllers.yaml')
    print("Hardware config path:", hardware_config)
    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, robot_description_semantic],
        namespace="",
    )

    # ros2_control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[hardware_config, robot_description],
        output='screen',
    )

    # Controller Spawners
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    trajectory_spawner = TimerAction(
        period=4.0,  # ensure ros2_control_node is fully initialized
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    "joint_trajectory_controller",
                    "--param-file", hardware_config,
                    "-c", "/controller_manager"
                ],
            )
        ]
    )

    # MoveIt move_group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_controllers,
            robot_description_kinematics, {'publish_robot_description_semantic': True},
            moveit_planners,
        ],
        namespace="",
    )

    # RViz
    rviz_config = os.path.join(description_dir, 'config', 'moveit.rviz')
    rviz_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )]
    )
    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        joint_state_spawner,
        trajectory_spawner,
        move_group_node,
        rviz_node,
    ])
