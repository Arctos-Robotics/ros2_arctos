from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Load the robot MoveIt configuration
    moveit_config = MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config").to_moveit_configs()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Get the path to the RViz configuration file
    #rviz_config_arg = DeclareLaunchArgument(
    #    "rviz_config",
    #    default_value="kinova_moveit_config_demo.rviz",
    #    description="RViz configuration file",
    #)
    #rviz_base = LaunchConfiguration("rviz_config")
    #rviz_config = PathJoinSubstitution(
    #    [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
    #)

    # Launch RViz
    rviz_config = PathJoinSubstitution([FindPackageShare('arctos_description'), 'rviz', 'plan_view_exec.rviz'])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    #static_tf = Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    name="static_transform_publisher",
    #    output="log",
    #    arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    #)

    # Publish TF
    #robot_state_publisher = Node(
    #    package="robot_state_publisher",
    #    executable="robot_state_publisher",
    #    name="robot_state_publisher",
    #    output="both",
    #    parameters=[moveit_config.robot_description],
    #)

    #ros2_controllers_path = PathJoinSubstitution([FindPackageShare("arctos_moveit"),"config","ros2_controller.yaml"])
    #ros2_control_node = Node(
    #    package="controller_manager",
    #    executable="ros2_control_node",
    #    parameters=[ros2_controllers_path],
    #    remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #    ],
    #    output="both",
    #)

    #joint_state_broadcaster_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=[
    #        "joint_state_broadcaster",
    #        "--controller-manager",
    #        "/controller_manager",
    #    ],
    #)
    

    arm_controller_spawner = Node(
         package="controller_manager",
        executable="spawner",
        arguments=["controller_manager"],
    )


    #delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #    event_handler=OnProcessExit(
    #        target_action=arm_controller_spawner,
    #        on_exit=[joint_state_broadcaster_spawner],
    #        )
    #)

    arctos_control_launch_path = PathJoinSubstitution([FindPackageShare('arctos_bringup'), 'launch', 'arctos_joint_trajectory_controller.launch.py'])
    arctos_control_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(arctos_control_launch_path))

    return LaunchDescription(
        [
            arctos_control_launch,
            rviz_node,
            #static_tf,
            #robot_state_publisher,
            run_move_group_node,
            #ros2_control_node,
            arm_controller_spawner,
            #delay_joint_state_broadcaster_after_robot_controller_spawner,
            #delay_rviz_after_joint_state_broadcaster_spawner,
        ]
    )