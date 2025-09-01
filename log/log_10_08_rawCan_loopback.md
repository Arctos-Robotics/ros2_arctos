[move_group-8] You can start planning now!
[move_group-8] 
[rviz2-7] [INFO] [1754847409.101812705] [move_group_interface]: Ready to take commands for planning group arctos_arm.
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[rviz2-7] [INFO] [1754848534.497506636] [rclcpp]: signal_handler(signum=2)
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 93235]
[INFO] [socket_can_sender_node_exe-5]: process has finished cleanly [pid 93243]
[move_group-8] [INFO] [1754848534.497507197] [rclcpp]: signal_handler(signum=2)
[socket_can_sender_node_exe-5] [INFO] [1754848534.497507478] [rclcpp]: signal_handler(signum=2)
[ros2_control_node-2] [INFO] [1754848534.497514641] [rclcpp]: signal_handler(signum=2)
[robot_state_publisher-1] [INFO] [1754848534.497527134] [rclcpp]: signal_handler(signum=2)
[socket_can_receiver_node_exe-4] [INFO] [1754848534.497508319] [rclcpp]: signal_handler(signum=2)
[ros2_control_node-2] [INFO] [1754848534.498040112] [controller_manager]: Shutdown request received....
[ros2_control_node-2] [INFO] [1754848534.498131863] [controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1754848534.498186616] [controller_manager]: Deactivating controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1754848534.498245385] [controller_manager]: Shutting down controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1754848534.498293405] [controller_manager]: Deactivating controller 'joint_trajectory_controller'
[ros2_control_node-2] [INFO] [1754848534.498344510] [controller_manager]: Shutting down controller 'joint_trajectory_controller'
[ros2_control_node-2] [INFO] [1754848534.498405715] [resource_manager]: 'deactivate' hardware 'arctos_interface' 
[ros2_control_node-2] [INFO] [1754848534.498432294] [arctos_hardware_interface]: Transitioning to INACTIVE state from active
[ros2_control_node-2] [INFO] [1754848534.498702098] [resource_manager]: Successful 'deactivate' of hardware 'arctos_interface'
[ros2_control_node-2] [INFO] [1754848534.498720683] [resource_manager]: 'shutdown' hardware 'arctos_interface' 
[ros2_control_node-2] [INFO] [1754848534.498729199] [resource_manager]: Successful 'shutdown' of hardware 'arctos_interface'
[ros2_control_node-2] [INFO] [1754848534.498739698] [controller_manager]: Shutting down the controller manager.
[ros2_control_node-2] [INFO] [1754848534.585865462] [arctos_hardware_interface]: Shutting down motor driver, stopping all motors...
[socket_can_receiver_node_exe-4] terminate called without an active exception
[rviz2-7] [WARN] [1754848534.504771510] [interactive_marker_display_101846222349760]: Server not available while running, resetting
[rviz2-7] [INFO] [1754848534.628918437] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[move_group-8] [INFO] [1754848534.538861220] [moveit.ros_planning_interface.moveit_cpp]: Deleting MoveItCpp
[move_group-8] [INFO] [1754848534.539823386] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopped publishing maintained planning scene.
[move_group-8] [INFO] [1754848534.542100658] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[move_group-8] [INFO] [1754848534.543192026] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[move_group-8] Warning: class_loader.ClassLoader: SEVERE WARNING!!! Attempting to unload library while objects created by this loader exist in the heap! You should delete your objects before attempting to unload the library or destroying the ClassLoader. The library will NOT be unloaded.
[move_group-8]          at line 127 in ./src/class_loader.cpp
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 93237]
[INFO] [move_group-8]: process has finished cleanly [pid 93326]
[move_group-8] 
[ERROR] [socket_can_receiver_node_exe-4]: process has died [pid 93241, exit code -6, cmd '/opt/ros/humble/lib/ros2_socketcan/socket_can_receiver_node_exe --ros-args --log-level motor_can_receiver:=error --ros-args -r __node:=motor_can_receiver -r __ns:=/ --params-file /tmp/launch_params__dn5ku49 -r /from_can_bus:=/from_motor_can_bus'].
[INFO] [rviz2-7]: process has finished cleanly [pid 93324]
arctos@arctos-HP-EliteDesk-705-G4-DM-35W-TAA:~/ros2_ws$ ros2 launch arctos_bringup arctos_bringup.launch.py 
[INFO] [launch]: All log files can be found below /home/arctos/.ros/log/2025-08-11-00-55-50-847898-arctos-HP-EliteDesk-705-G4-DM-35W-TAA-102437
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Launching Arctos Bringup with RViz...
[INFO] [robot_state_publisher-1]: process started with pid [102448]
[INFO] [ros2_control_node-2]: process started with pid [102450]
[INFO] [spawner-3]: process started with pid [102452]
[INFO] [socket_can_receiver_node_exe-4]: process started with pid [102454]
[INFO] [socket_can_sender_node_exe-5]: process started with pid [102456]
[socket_can_sender_node_exe-5] [INFO] [1754848551.357628456] [motor_can_sender]: interface: can0
[socket_can_sender_node_exe-5] [INFO] [1754848551.357825394] [motor_can_sender]: can fd enabled: false
[socket_can_sender_node_exe-5] [INFO] [1754848551.357844149] [motor_can_sender]: timeout(s): 0.010000
[robot_state_publisher-1] [INFO] [1754848551.362148958] [robot_state_publisher]: got segment Gripper_1
[robot_state_publisher-1] [INFO] [1754848551.362284000] [robot_state_publisher]: got segment Left_jaw_1
[robot_state_publisher-1] [INFO] [1754848551.362299910] [robot_state_publisher]: got segment Link_1_1
[robot_state_publisher-1] [INFO] [1754848551.362310460] [robot_state_publisher]: got segment Link_2_1
[robot_state_publisher-1] [INFO] [1754848551.362320318] [robot_state_publisher]: got segment Link_3_1
[robot_state_publisher-1] [INFO] [1754848551.362394006] [robot_state_publisher]: got segment Link_4_1
[robot_state_publisher-1] [INFO] [1754848551.362405537] [robot_state_publisher]: got segment Link_5_1
[robot_state_publisher-1] [INFO] [1754848551.362415135] [robot_state_publisher]: got segment Link_6_1
[robot_state_publisher-1] [INFO] [1754848551.362425444] [robot_state_publisher]: got segment Right_jaw_1
[robot_state_publisher-1] [INFO] [1754848551.362434471] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1754848551.362443468] [robot_state_publisher]: got segment world
[ros2_control_node-2] [INFO] [1754848551.382768896] [controller_manager]: Subscribing to '~/robot_description' topic for robot description file.
[ros2_control_node-2] [INFO] [1754848551.383746962] [controller_manager]: update rate is 20 Hz
[ros2_control_node-2] [INFO] [1754848551.383797747] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [WARN] [1754848551.383970179] [controller_manager]: No real-time kernel detected on this system. See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
[ros2_control_node-2] [INFO] [1754848551.385051148] [controller_manager]: Received robot description file.
[ros2_control_node-2] [INFO] [1754848551.385428382] [resource_manager]: Loading hardware 'arctos_interface' 
[ros2_control_node-2] [INFO] [1754848551.396036653] [resource_manager]: Initialize hardware 'arctos_interface' 
[ros2_control_node-2] [INFO] [1754848551.396806231] [arctos_hardware_interface]: Configured joint X_joint with motor_id 1
[ros2_control_node-2] [INFO] [1754848551.397102414] [arctos_hardware_interface]: Configured joint Y_joint with motor_id 2
[ros2_control_node-2] [INFO] [1754848551.398149409] [arctos_hardware_interface]: Configured joint Z_joint with motor_id 3
[ros2_control_node-2] [INFO] [1754848551.398604779] [arctos_hardware_interface]: Configured joint A_joint with motor_id 4
[ros2_control_node-2] [INFO] [1754848551.399012381] [arctos_hardware_interface]: Configured joint B_joint with motor_id 5
[ros2_control_node-2] [INFO] [1754848551.399492988] [arctos_hardware_interface]: Configured joint C_joint with motor_id 6
[ros2_control_node-2] [INFO] [1754848551.399605037] [resource_manager]: Successful initialization of hardware 'arctos_interface'
[ros2_control_node-2] [INFO] [1754848551.399900088] [resource_manager]: 'configure' hardware 'arctos_interface' 
[ros2_control_node-2] [INFO] [1754848551.399914305] [arctos_hardware_interface]: Transitioning to CONFIGURE state from unconfigured
[ros2_control_node-2] [INFO] [1754848551.400173779] [arctos_hardware_interface]: Added joint X_joint with motor ID 1 and gear ratio 13.50:1
[ros2_control_node-2] [INFO] [1754848551.400443944] [arctos_hardware_interface]: Updated limits for joint X_joint: pos=[-3.03, 2.75], vel=23.27, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400470834] [arctos_hardware_interface]: Set joint limits for joint X_joint of motor 1: pos=[-3.03, 2.75], vel=23.27, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400484961] [arctos_hardware_interface]: Initialized motor for joint X_joint with ID 1 and gear ratio 13.50:1
[ros2_control_node-2] [INFO] [1754848551.400520297] [arctos_hardware_interface]: Added joint Y_joint with motor ID 2 and gear ratio 150.00:1
[ros2_control_node-2] [INFO] [1754848551.400551605] [arctos_hardware_interface]: Updated limits for joint Y_joint: pos=[-0.85, 2.01], vel=2.09, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400566212] [arctos_hardware_interface]: Set joint limits for joint Y_joint of motor 2: pos=[-0.85, 2.01], vel=2.09, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400578064] [arctos_hardware_interface]: Initialized motor for joint Y_joint with ID 2 and gear ratio 150.00:1
[ros2_control_node-2] [INFO] [1754848551.400597481] [arctos_hardware_interface]: Added joint Z_joint with motor ID 3 and gear ratio 150.00:1
[ros2_control_node-2] [INFO] [1754848551.400626755] [arctos_hardware_interface]: Updated limits for joint Z_joint: pos=[-1.57, 0.96], vel=2.09, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400645611] [arctos_hardware_interface]: Set joint limits for joint Z_joint of motor 3: pos=[-1.57, 0.96], vel=2.09, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400661470] [arctos_hardware_interface]: Initialized motor for joint Z_joint with ID 3 and gear ratio 150.00:1
[ros2_control_node-2] [INFO] [1754848551.400688280] [arctos_hardware_interface]: Added joint A_joint with motor ID 4 and gear ratio 48.00:1
[ros2_control_node-2] [INFO] [1754848551.400719749] [arctos_hardware_interface]: Updated limits for joint A_joint: pos=[-1.57, 1.57], vel=6.54, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400733735] [arctos_hardware_interface]: Set joint limits for joint A_joint of motor 4: pos=[-1.57, 1.57], vel=6.54, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400745126] [arctos_hardware_interface]: Initialized motor for joint A_joint with ID 4 and gear ratio 48.00:1
[ros2_control_node-2] [INFO] [1754848551.400764513] [arctos_hardware_interface]: Added joint B_joint with motor ID 5 and gear ratio 67.82:1
[ros2_control_node-2] [INFO] [1754848551.400791784] [arctos_hardware_interface]: Updated limits for joint B_joint: pos=[-1.55, 1.55], vel=4.63, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400805189] [arctos_hardware_interface]: Set joint limits for joint B_joint of motor 5: pos=[-1.55, 1.55], vel=4.63, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400869058] [arctos_hardware_interface]: Initialized motor for joint B_joint with ID 5 and gear ratio 67.82:1
[ros2_control_node-2] [INFO] [1754848551.400903101] [arctos_hardware_interface]: Added joint C_joint with motor ID 6 and gear ratio 67.82:1
[ros2_control_node-2] [INFO] [1754848551.400945000] [arctos_hardware_interface]: Updated limits for joint C_joint: pos=[-3.14, 3.14], vel=4.63, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400961621] [arctos_hardware_interface]: Set joint limits for joint C_joint of motor 6: pos=[-3.14, 3.14], vel=4.63, acc=255.00
[ros2_control_node-2] [INFO] [1754848551.400973333] [arctos_hardware_interface]: Initialized motor for joint C_joint with ID 6 and gear ratio 67.82:1
[ros2_control_node-2] [INFO] [1754848551.400992068] [resource_manager]: Successful 'configure' of hardware 'arctos_interface'
[ros2_control_node-2] [INFO] [1754848551.401006975] [resource_manager]: 'activate' hardware 'arctos_interface' 
[ros2_control_node-2] [INFO] [1754848551.401012736] [arctos_hardware_interface]: Transitioning to ACTIVE state from inactive
[ros2_control_node-2] [INFO] [1754848551.401022474] [arctos_hardware_interface]: Enabling motor for joint X_joint
[ros2_control_node-2] [INFO] [1754848551.401057650] [arctos_hardware_interface]: Enabling motor for joint Y_joint
[ros2_control_node-2] [INFO] [1754848551.401098957] [arctos_hardware_interface]: Enabling motor for joint Z_joint
[ros2_control_node-2] [INFO] [1754848551.401117732] [arctos_hardware_interface]: Enabling motor for joint A_joint
[ros2_control_node-2] [INFO] [1754848551.401139603] [arctos_hardware_interface]: Enabling motor for joint B_joint
[ros2_control_node-2] [INFO] [1754848551.401165682] [arctos_hardware_interface]: Enabling motor for joint C_joint
[ros2_control_node-2] [INFO] [1754848551.401196600] [resource_manager]: Successful 'activate' of hardware 'arctos_interface'
[ros2_control_node-2] [INFO] [1754848551.434376032] [arctos_hardware_interface]: Initialized last command vectors.
[ros2_control_node-2] [INFO] [1754848551.484318460] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.489740716] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.494931009] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.500147450] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.505428172] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.510639284] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.534276146] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.539486967] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.544684844] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.549916404] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.555149036] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.560375717] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.584218663] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.589434985] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.594680521] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.599912782] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.605165852] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.610435423] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.634250398] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.639507024] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.644759543] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.650015088] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.655313112] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.660574839] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848551.684515758] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848551.684569629] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848551.684590127] [arctos_hardware_interface]: Byte 2: 0x00
[ros2_control_node-2] [WARN] [1754848551.684606147] [arctos_hardware_interface]: Byte 3: 0x00
[ros2_control_node-2] [WARN] [1754848551.684619662] [arctos_hardware_interface]: Byte 4: 0x00
[ros2_control_node-2] [WARN] [1754848551.684632727] [arctos_hardware_interface]: Byte 5: 0x00
[ros2_control_node-2] [WARN] [1754848551.684645400] [arctos_hardware_interface]: Byte 6: 0x00
[ros2_control_node-2] [WARN] [1754848551.684656070] [arctos_hardware_interface]: Byte 7: 0x00
[ros2_control_node-2] [INFO] [1754848551.684673352] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848551.684700002] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848551.684748823] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.689913829] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.695146952] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.700484871] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.705713675] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.710984539] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.712752711] [controller_manager]: Loading controller 'joint_state_broadcaster'
[ros2_control_node-2] [WARN] [1754848551.734279602] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848551.734341067] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848551.734358299] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848551.734373918] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848551.734398204] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848551.734415155] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848551.734429572] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848551.734444380] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848551.734469777] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848551.734495776] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848551.734533045] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.739744508] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.744967231] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.750197850] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.755442674] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.760743213] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848551.784290798] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848551.784345721] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848551.784360087] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848551.784372972] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848551.784404040] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848551.784417214] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848551.784429247] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848551.784441610] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848551.784454063] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848551.784472998] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848551.784509767] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[spawner-3] [INFO] [1754848551.789048393] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[ros2_control_node-2] [INFO] [1754848551.789720067] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.790596344] [controller_manager]: Configuring controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1754848551.790800435] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[ros2_control_node-2] [INFO] [1754848551.795026988] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.800241757] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.805552284] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.810784535] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848551.834318094] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848551.834378677] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848551.834413392] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848551.834430113] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848551.834446213] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848551.834461492] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848551.834475458] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848551.834490646] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848551.834506746] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848551.834529068] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848551.834562160] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.839752433] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.844978663] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.850157624] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.855339111] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.860615755] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848551.884287031] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848551.884347494] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848551.884408829] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848551.884424227] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848551.884436981] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848551.884449625] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848551.884462499] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848551.884475623] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848551.884489179] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848551.884508956] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848551.884537519] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.889735666] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.894992343] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.900222511] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.905435245] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.910665313] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[spawner-3] [INFO] [1754848551.917536444] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[ros2_control_node-2] [WARN] [1754848551.934217386] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848551.934259254] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848551.934270916] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848551.934279953] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848551.934287397] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848551.934294861] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848551.934302164] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848551.934310159] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848551.934317764] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848551.934355945] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848551.934376804] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.939530969] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.944730399] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.949931202] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.955129469] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.960407767] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848551.984401164] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848551.984480823] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848551.984492264] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848551.984503395] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848551.984512983] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848551.984522381] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848551.984531638] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848551.984541607] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848551.984551836] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848551.984572124] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848551.984603192] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.989801249] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848551.995081921] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.000319452] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.005549248] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.010746895] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848552.034264014] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.034318866] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848552.034332592] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.034344524] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.034355996] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.034367016] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.034378257] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848552.034410137] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848552.034422630] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.034440413] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848552.034465290] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.039636056] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.044852699] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.050092534] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.055324264] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.060548220] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848552.084291000] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.084350031] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848552.084362995] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.084374697] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.084430210] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.084442974] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.084453895] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848552.084464715] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848552.084508346] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.084528775] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848552.084569671] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.089779931] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.095071373] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.100353187] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.105652974] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.110983820] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848552.134285747] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.134340589] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848552.134354135] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.134366648] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.134378540] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.134398197] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.134409227] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848552.134420308] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848552.134431699] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.134448791] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848552.134473387] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.139665724] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[INFO] [spawner-3]: process has finished cleanly [pid 102452]
[INFO] [spawner-6]: process started with pid [102524]
[ros2_control_node-2] [INFO] [1754848552.144928723] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.150182244] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.155430755] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.160688254] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848552.184310830] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.184366544] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848552.184402821] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.184419923] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.184432777] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.184444620] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.184456381] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848552.184468945] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848552.184481859] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.184501125] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848552.184528987] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.189711916] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.194974084] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.200241110] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.205497336] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.210751037] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848552.234349969] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.234445487] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848552.234468380] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.234489279] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.234507754] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.234526569] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.234583786] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848552.234607229] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848552.234629130] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.234656531] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848552.234692338] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.239909201] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.245170917] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.250423617] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.255688229] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.260937532] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848552.284269024] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.284332012] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848552.284414005] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.284435645] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.284452336] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.284468306] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.284484286] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848552.284500576] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848552.284517117] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.284542004] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848552.284577079] [arctos_hardware_interface]: Joint A_joint (Motor ID: 4) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.289772091] [arctos_hardware_interface]: Joint B_joint (Motor ID: 5) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.295032415] [arctos_hardware_interface]: Joint C_joint (Motor ID: 6) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.300283682] [arctos_hardware_interface]: Joint X_joint (Motor ID: 1) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.305534147] [arctos_hardware_interface]: Joint Y_joint (Motor ID: 2) is_moving: false
[ros2_control_node-2] [INFO] [1754848552.310800051] [arctos_hardware_interface]: Joint Z_joint (Motor ID: 3) is_moving: false
[ros2_control_node-2] [WARN] [1754848552.334293836] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.334352526] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848552.334369197] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.334393402] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.334408781] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.334422537] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.334436493] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848552.334451270] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848552.334465778] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.334487238] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [WARN] [1754848552.384290777] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.384348865] [arctos_hardware_interface]: Byte 1: 0xFF
[ros2_control_node-2] [WARN] [1754848552.384365236] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.384380004] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.384409809] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.384424336] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.384438603] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848552.384453451] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848552.384467938] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [INFO] [1754848552.384516589] [arctos_hardware_interface]: Processing encoder response
[ros2_control_node-2] [WARN] [1754848552.384587962] [arctos_hardware_interface]: Discarding out-of-range motor angle: 281474976594182.00 degrees for motor ID 2
[ros2_control_node-2] [WARN] [1754848552.434261989] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.434343962] [arctos_hardware_interface]: Byte 1: 0x34
[ros2_control_node-2] [WARN] [1754848552.434356466] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.434366885] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.434376783] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.434423150] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.434433169] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848552.434457033] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848552.434468014] [arctos_hardware_interface]: CAN message received - ID: 3, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.434483733] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [WARN] [1754848552.484413558] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.484497445] [arctos_hardware_interface]: Byte 1: 0x35
[ros2_control_node-2] [WARN] [1754848552.484512333] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.484525517] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.484537961] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.484550484] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.484563338] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848552.484616547] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848552.484629602] [arctos_hardware_interface]: CAN message received - ID: 4, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.484653326] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [INFO] [1754848552.507550307] [controller_manager]: Loading controller 'joint_trajectory_controller'
[ros2_control_node-2] [WARN] [1754848552.519151793] [joint_trajectory_controller]: [Deprecated]: "allow_nonzero_velocity_at_trajectory_end" is set to true. The default behavior will change to false.
[ros2_control_node-2] [WARN] [1754848552.534243266] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.534290415] [arctos_hardware_interface]: Byte 1: 0x36
[ros2_control_node-2] [WARN] [1754848552.534300934] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.534309931] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.534318387] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.534326482] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.534334817] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848552.534368641] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848552.534378028] [arctos_hardware_interface]: CAN message received - ID: 5, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.534402394] [arctos_hardware_interface]: Received incomplete encoder response
[spawner-6] [INFO] [1754848552.555448037] [spawner_joint_trajectory_controller]: Loaded joint_trajectory_controller
[ros2_control_node-2] [INFO] [1754848552.556498919] [controller_manager]: Configuring controller 'joint_trajectory_controller'
[ros2_control_node-2] [INFO] [1754848552.556728498] [joint_trajectory_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[ros2_control_node-2] [INFO] [1754848552.556758023] [joint_trajectory_controller]: Command interfaces are [position velocity] and state interfaces are [position velocity].
[ros2_control_node-2] [INFO] [1754848552.556785154] [joint_trajectory_controller]: Using 'splines' interpolation method.
[ros2_control_node-2] [INFO] [1754848552.557302700] [joint_trajectory_controller]: Controller state will be published at 25.00 Hz.
[ros2_control_node-2] [INFO] [1754848552.560172069] [joint_trajectory_controller]: Goals with partial set of joints are allowed
[ros2_control_node-2] [INFO] [1754848552.560208727] [joint_trajectory_controller]: Action status changes will be monitored at 10.00 Hz.
[ros2_control_node-2] [WARN] [1754848552.584276585] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.584330966] [arctos_hardware_interface]: Byte 1: 0x37
[ros2_control_node-2] [WARN] [1754848552.584345634] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.584358478] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.584371021] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.584397972] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.584441152] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848552.584454717] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848552.584467582] [arctos_hardware_interface]: CAN message received - ID: 6, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.584487178] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [WARN] [1754848552.634308280] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.634365036] [arctos_hardware_interface]: Byte 1: 0x32
[ros2_control_node-2] [WARN] [1754848552.634378612] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.634402626] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.634414639] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.634426301] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.634437652] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848552.634449494] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848552.634461146] [arctos_hardware_interface]: CAN message received - ID: 1, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.634479490] [arctos_hardware_interface]: Received incomplete encoder response
[ros2_control_node-2] [WARN] [1754848552.684279833] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.684330568] [arctos_hardware_interface]: Byte 1: 0x33
[ros2_control_node-2] [WARN] [1754848552.684342240] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.684352248] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.684361796] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.684371615] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.684395760] [arctos_hardware_interface]: Byte 6: 0x06
[ros2_control_node-2] [WARN] [1754848552.684406810] [arctos_hardware_interface]: Byte 7: 0x6D
[ros2_control_node-2] [INFO] [1754848552.684417019] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.684432498] [arctos_hardware_interface]: Received incomplete encoder response
[spawner-6] [INFO] [1754848552.685699014] [spawner_joint_trajectory_controller]: Configured and activated joint_trajectory_controller
[ros2_control_node-2] [WARN] [1754848552.734289918] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.734342797] [arctos_hardware_interface]: Byte 1: 0xFF
[ros2_control_node-2] [WARN] [1754848552.734354178] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.734364327] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.734373825] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.734391127] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.734401156] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848552.734411164] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848552.734420943] [arctos_hardware_interface]: CAN message received - ID: 2, Command: 0x31
[ros2_control_node-2] [INFO] [1754848552.734436301] [arctos_hardware_interface]: Processing encoder response
[ros2_control_node-2] [WARN] [1754848552.734456379] [arctos_hardware_interface]: Discarding out-of-range motor angle: 281474976594181.00 degrees for motor ID 2
[ros2_control_node-2] [WARN] [1754848552.784275998] [arctos_hardware_interface]: Byte 0: 0x31
[ros2_control_node-2] [WARN] [1754848552.784324789] [arctos_hardware_interface]: Byte 1: 0x34
[ros2_control_node-2] [WARN] [1754848552.784336040] [arctos_hardware_interface]: Byte 2: 0xFF
[ros2_control_node-2] [WARN] [1754848552.784345498] [arctos_hardware_interface]: Byte 3: 0xFF
[ros2_control_node-2] [WARN] [1754848552.784354505] [arctos_hardware_interface]: Byte 4: 0xFE
[ros2_control_node-2] [WARN] [1754848552.784363742] [arctos_hardware_interface]: Byte 5: 0x39
[ros2_control_node-2] [WARN] [1754848552.784372559] [arctos_hardware_interface]: Byte 6: 0x05
[ros2_control_node-2] [WARN] [1754848552.784395081] [arctos_hardware_interface]: Byte 7: 0x6C
[ros2_control_node-2] [INFO] [1754848552.784404869] [arctos_hardware_interface]: CAN message received - ID: 3, Command: 0x31
[ros2_control_node-2] [WARN] [1754848552.784420177] [arctos_hardware_interface]: Received incomplete encoder response
[INFO] [spawner-6]: process has finished cleanly [pid 102524]
[INFO] [rviz2-7]: process started with pid [102552]
[INFO] [move_group-8]: process started with pid [102554]
[move_group-8] [INFO] [1754848553.193136188] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00601945 seconds
[move_group-8] [INFO] [1754848553.193211178] [moveit_robot_model.robot_model]: Loading robot model 'arctos'...
[rviz2-7] [INFO] [1754848553.630498261] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-7] [INFO] [1754848553.630682365] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
[rviz2-7] [INFO] [1754848553.658008949] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-7] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-7]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[move_group-8] [INFO] [1754848555.222672320] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-8] [INFO] [1754848555.222888033] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[move_group-8] [INFO] [1754848555.223397654] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-8] [INFO] [1754848555.223894212] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[move_group-8] [INFO] [1754848555.223919229] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[move_group-8] [INFO] [1754848555.224774456] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-8] [INFO] [1754848555.224802308] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-8] [INFO] [1754848555.225242400] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[move_group-8] [INFO] [1754848555.225722667] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-8] [WARN] [1754848555.227093597] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-8] [ERROR] [1754848555.227118594] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[rviz2-7] [ERROR] [1754848556.865646169] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-7] [INFO] [1754848556.881328648] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-7] [WARN] [1754848556.912105067] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-7] [INFO] [1754848557.332373232] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.42107 seconds
[rviz2-7] [INFO] [1754848557.332433654] [moveit_robot_model.robot_model]: Loading robot model 'arctos'...
[rviz2-7] [WARN] [1754848558.043973020] [moveit_ros.robot_model_loader]: No kinematics plugins defined. Fill and load kinematics.yaml!
[rviz2-7] [INFO] [1754848559.484718236] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-7] [INFO] [1754848559.486808931] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[move_group-8] [INFO] [1754848561.026449376] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'pilz_industrial_motion_planner'
[move_group-8] [INFO] [1754848561.033555328] [moveit.pilz_industrial_motion_planner.joint_limits_aggregator]: Reading limits from namespace robot_description_planning
[move_group-8] [INFO] [1754848561.044094813] [moveit.pilz_industrial_motion_planner]: Available plugins: pilz_industrial_motion_planner/PlanningContextLoaderCIRC pilz_industrial_motion_planner/PlanningContextLoaderLIN pilz_industrial_motion_planner/PlanningContextLoaderPTP 
[move_group-8] [INFO] [1754848561.044118847] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderCIRC
[move_group-8] [INFO] [1754848561.045869046] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [CIRC]
[move_group-8] [INFO] [1754848561.045891969] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderLIN
[move_group-8] [INFO] [1754848561.046944916] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [LIN]
[move_group-8] [INFO] [1754848561.046965805] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderPTP
[move_group-8] [INFO] [1754848561.048034210] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [PTP]
[move_group-8] [INFO] [1754848561.048079064] [moveit.ros_planning.planning_pipeline]: Using planning interface 'Pilz Industrial Motion Planner'
[move_group-8] [INFO] [1754848561.050389219] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'chomp'
[move_group-8] [INFO] [1754848561.059499915] [moveit.ros_planning.planning_pipeline]: Using planning interface 'CHOMP'
[move_group-8] [INFO] [1754848561.066490972] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.path_tolerance' was not set. Using default value: 0.100000
[move_group-8] [INFO] [1754848561.066513104] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.resample_dt' was not set. Using default value: 0.100000
[move_group-8] [INFO] [1754848561.066520417] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.min_angle_change' was not set. Using default value: 0.001000
[move_group-8] [INFO] [1754848561.066546787] [moveit_ros.fix_workspace_bounds]: Param 'chomp.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-8] [INFO] [1754848561.066563538] [moveit_ros.fix_start_state_bounds]: Param 'chomp.start_state_max_bounds_error' was set to 0.100000
[move_group-8] [INFO] [1754848561.066569990] [moveit_ros.fix_start_state_bounds]: Param 'chomp.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-8] [INFO] [1754848561.066619122] [moveit_ros.fix_start_state_collision]: Param 'chomp.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-8] [INFO] [1754848561.066626095] [moveit_ros.fix_start_state_collision]: Param 'chomp.jiggle_fraction' was set to 0.050000
[move_group-8] [INFO] [1754848561.066637095] [moveit_ros.fix_start_state_collision]: Param 'chomp.max_sampling_attempts' was not set. Using default value: 100
[move_group-8] [INFO] [1754848561.066652274] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-8] [INFO] [1754848561.066658335] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-8] [INFO] [1754848561.066662342] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-8] [INFO] [1754848561.066666821] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-8] [INFO] [1754848561.066672191] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-8] [INFO] [1754848561.066677280] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-8] [INFO] [1754848561.067740897] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'ompl'
[move_group-8] [INFO] [1754848561.083957946] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[move_group-8] [INFO] [1754848561.089222499] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.path_tolerance' was not set. Using default value: 0.100000
[move_group-8] [INFO] [1754848561.089250561] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.resample_dt' was not set. Using default value: 0.100000
[move_group-8] [INFO] [1754848561.089256532] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.min_angle_change' was not set. Using default value: 0.001000
[move_group-8] [INFO] [1754848561.089285065] [moveit_ros.fix_workspace_bounds]: Param 'ompl.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-8] [INFO] [1754848561.089305884] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_bounds_error' was set to 0.100000
[move_group-8] [INFO] [1754848561.089312497] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-8] [INFO] [1754848561.089328006] [moveit_ros.fix_start_state_collision]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-8] [INFO] [1754848561.089334698] [moveit_ros.fix_start_state_collision]: Param 'ompl.jiggle_fraction' was set to 0.050000
[move_group-8] [INFO] [1754848561.089341040] [moveit_ros.fix_start_state_collision]: Param 'ompl.max_sampling_attempts' was not set. Using default value: 100
[move_group-8] [INFO] [1754848561.089355808] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-8] [INFO] [1754848561.089363562] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-8] [INFO] [1754848561.089368561] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-8] [INFO] [1754848561.089372859] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-8] [INFO] [1754848561.089376907] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-8] [INFO] [1754848561.089392246] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-8] [INFO] [1754848561.134433520] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for joint_trajectory_controller
[move_group-8] [INFO] [1754848561.134740754] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-8] [INFO] [1754848561.134819611] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-8] [INFO] [1754848561.135133367] [moveit_ros.trajectory_execution_manager]: Trajectory execution is managing controllers
[move_group-8] [INFO] [1754848561.135191446] [move_group.move_group]: MoveGroup debug mode is ON
[move_group-8] [INFO] [1754848561.161198790] [move_group.move_group]: 
[move_group-8] 
[move_group-8] ********************************************************
[move_group-8] * MoveGroup using: 
[move_group-8] *     - ApplyPlanningSceneService
[move_group-8] *     - ClearOctomapService
[move_group-8] *     - CartesianPathService
[move_group-8] *     - ExecuteTrajectoryAction
[move_group-8] *     - GetPlanningSceneService
[move_group-8] *     - KinematicsService
[move_group-8] *     - MoveAction
[move_group-8] *     - MotionPlanService
[move_group-8] *     - QueryPlannersService
[move_group-8] *     - StateValidationService
[move_group-8] ********************************************************
[move_group-8] 
[move_group-8] [INFO] [1754848561.161278229] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[move_group-8] [INFO] [1754848561.161293186] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete
[move_group-8] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-8] Loading 'move_group/ClearOctomapService'...
[move_group-8] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-8] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-8] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-8] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-8] Loading 'move_group/MoveGroupMoveAction'...
[move_group-8] Loading 'move_group/MoveGroupPlanService'...
[move_group-8] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-8] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-8] 
[move_group-8] You can start planning now!
[move_group-8] 
[rviz2-7] [INFO] [1754848561.267438667] [interactive_marker_display_97095959806192]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-7] [INFO] [1754848561.289332107] [moveit_ros_robot_interaction.robot_interaction]: No active joints or end effectors found for group 'arctos_arm'. Make sure that kinematics.yaml is loaded in this node's namespace.
[rviz2-7] [INFO] [1754848561.295267623] [moveit_ros_visualization.motion_planning_frame]: group arctos_arm
[rviz2-7] [INFO] [1754848561.295297880] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'arctos_arm' in namespace ''
[rviz2-7] [INFO] [1754848561.307113528] [move_group_interface]: Ready to take commands for planning group arctos_arm.
[rviz2-7] [INFO] [1754848561.315664699] [interactive_marker_display_97095959806192]: Sending request for interactive markers
[rviz2-7] [INFO] [1754848561.332394256] [interactive_marker_display_97095959806192]: Service response received for initialization
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[rviz2-7] [INFO] [1754848577.906319791] [rclcpp]: signal_handler(signum=2)
[INFO] [socket_can_sender_node_exe-5]: process has finished cleanly [pid 102456]
[move_group-8] [INFO] [1754848577.906320032] [rclcpp]: signal_handler(signum=2)
[ros2_control_node-2] [INFO] [1754848577.906321324] [rclcpp]: signal_handler(signum=2)
[ros2_control_node-2] [INFO] [1754848577.906478668] [controller_manager]: Shutdown request received....
[robot_state_publisher-1] [INFO] [1754848577.906328728] [rclcpp]: signal_handler(signum=2)
[socket_can_receiver_node_exe-4] [INFO] [1754848577.906322867] [rclcpp]: signal_handler(signum=2)
[socket_can_receiver_node_exe-4] terminate called without an active exception
[socket_can_sender_node_exe-5] [INFO] [1754848577.906319741] [rclcpp]: signal_handler(signum=2)
[ros2_control_node-2] [INFO] [1754848577.906572844] [controller_manager]: Shutting down all controllers in the controller manager.
[ros2_control_node-2] [INFO] [1754848577.906611186] [controller_manager]: Deactivating controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1754848577.906665527] [controller_manager]: Shutting down controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1754848577.906719618] [controller_manager]: Deactivating controller 'joint_trajectory_controller'
[ros2_control_node-2] [INFO] [1754848577.907270347] [controller_manager]: Shutting down controller 'joint_trajectory_controller'
[ros2_control_node-2] [INFO] [1754848577.907352270] [resource_manager]: 'deactivate' hardware 'arctos_interface' 
[ros2_control_node-2] [INFO] [1754848577.907399348] [arctos_hardware_interface]: Transitioning to INACTIVE state from active
[ros2_control_node-2] [INFO] [1754848577.907672619] [resource_manager]: Successful 'deactivate' of hardware 'arctos_interface'
[ros2_control_node-2] [INFO] [1754848577.907695161] [resource_manager]: 'shutdown' hardware 'arctos_interface' 
[ros2_control_node-2] [INFO] [1754848577.907706071] [resource_manager]: Successful 'shutdown' of hardware 'arctos_interface'
[ros2_control_node-2] [INFO] [1754848577.907717593] [controller_manager]: Shutting down the controller manager.
[ros2_control_node-2] [INFO] [1754848578.004197283] [arctos_hardware_interface]: Shutting down motor driver, stopping all motors...
[rviz2-7] [WARN] [1754848577.907760753] [interactive_marker_display_97095959806192]: Server not available while running, resetting
[rviz2-7] [INFO] [1754848578.036369103] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[move_group-8] [INFO] [1754848577.947293794] [moveit.ros_planning_interface.moveit_cpp]: Deleting MoveItCpp
[move_group-8] [INFO] [1754848577.948259217] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopped publishing maintained planning scene.
[move_group-8] [INFO] [1754848577.950532734] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[move_group-8] [INFO] [1754848577.951647116] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[move_group-8] Warning: class_loader.ClassLoader: SEVERE WARNING!!! Attempting to unload library while objects created by this loader exist in the heap! You should delete your objects before attempting to unload the library or destroying the ClassLoader. The library will NOT be unloaded.
[move_group-8]          at line 127 in ./src/class_loader.cpp
[INFO] [robot_state_publisher-1]: process has finished cleanly [pid 102448]
[INFO] [ros2_control_node-2]: process has finished cleanly [pid 102450]
[INFO] [move_group-8]: process has finished cleanly [pid 102554]
[move_group-8] 
[ERROR] [socket_can_receiver_node_exe-4]: process has died [pid 102454, exit code -6, cmd '/opt/ros/humble/lib/ros2_socketcan/socket_can_receiver_node_exe --ros-args --log-level motor_can_receiver:=error --ros-args -r __node:=motor_can_receiver -r __ns:=/ --params-file /tmp/launch_params_9l55fa4x -r /from_can_bus:=/from_motor_can_bus'].
[INFO] [rviz2-7]: process has finished cleanly [pid 102552]
