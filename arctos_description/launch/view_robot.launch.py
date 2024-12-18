import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    pkg_dir = get_package_share_directory('arctos_description')
    
    # Get URDF via xacro
    urdf_file = os.path.join(pkg_dir, 'urdf', 'arctos.xacro')
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            urdf_file
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Load RViz
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'arctos.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Joint State Publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    return LaunchDescription([
        robot_state_pub_node,
        joint_state_publisher_node,
        rviz_node
    ])