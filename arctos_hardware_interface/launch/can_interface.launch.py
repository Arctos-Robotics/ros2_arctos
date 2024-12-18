from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition

def generate_launch_description():
    """
    Start the CAN receiver nodes to connect to the Motors
    """
    # Launch arguments
    launch_arguments = []

    motor_can_interface_arg = DeclareLaunchArgument(
        name="motor_can_interface",
        default_value="can0",
        description="Which can interface to use for the motor CAN network, in case of multiple can networks",
    )
    launch_arguments.append(motor_can_interface_arg)

    motor_can_send_topic_arg = DeclareLaunchArgument(
        name="motor_can_send_topic",
        default_value="/to_motor_can_bus",
        description="Topic to use for sending to the motor network CAN messages",
    )
    launch_arguments.append(motor_can_send_topic_arg)

    motor_can_recv_topic_arg = DeclareLaunchArgument(
        name="motor_can_recv_topic",
        default_value="/from_motor_can_bus",
        description="Topic to use for receiving from the sensor network CAN messages",
    )
    launch_arguments.append(motor_can_recv_topic_arg)

    # Lifecycle and nodes

    lifecycle_nodes: list[LifecycleNode] = []
    launch_executables = []

    # Receiver node
        
    motor_can_receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='motor_can_receiver',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': LaunchConfiguration(motor_can_interface_arg.name),
        }],
        remappings={
            "/from_can_bus": LaunchConfiguration(motor_can_recv_topic_arg.name),
        }.items(),
        output='screen',
    )
    lifecycle_nodes.append(motor_can_receiver_node)

    ## Send node

    motor_can_sender_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='motor_can_sender',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': LaunchConfiguration(motor_can_interface_arg.name),
        }],
        remappings={
            "/to_can_bus": LaunchConfiguration(motor_can_send_topic_arg.name),
        }.items(),
        output='screen',
    )
    lifecycle_nodes.append(motor_can_sender_node)

    for node in lifecycle_nodes:
        # Launch the lifecycle node
        launch_executables.append(node)

        # Configure after process is started
        launch_executables.append(
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node,
                    on_start=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(node),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            ),
                        ),
                    ],
                ),
            )
        )
        # Activate after configure is done
        launch_executables.append(
            RegisterEventHandler(
                event_handler=OnStateTransition(
                    target_lifecycle_node=node,
                    start_state='configuring',
                    goal_state='inactive',
                    entities=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(node),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            ),
                        ),
                    ],
                ),
            )
        )

    return LaunchDescription([

        *launch_arguments,
        *launch_executables
    ])    