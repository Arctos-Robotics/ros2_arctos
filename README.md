# ROS2 Arctos
![Discord](https://img.shields.io/discord/1099629962618748958?logo=discord&logoColor=%23FFFFFF&logoSize=auto)


## **Warning** 

This project is still under development and is not yet ready for production use. We are actively working on improving the project and adding new features. If you would like to contribute, please see the [Contributing Guidelines](CONTRIBUTING.md).

**USE AT YOUR OWN RISK**.

Only use this project if you have:
- Ensured the KY-003/WSH231 hall effect sensors have been tested and are working correctly.
    - ⚠️ If the hall effect sensors are swapped (meaning the sensor for the home position is connected on the pin of the sensor for the opposite limit position), the `set_zero_position.py` script will not work correctly.
    - ⚠️ If the hall effect sensors are not being triggered, the arm will not stop moving when it reaches the limit positions.
        - Pressing "ESC" while running the `set_zero_position.py` script will stop the robot from moving. (*You must be running the script in a terminal that has focus for this to work.*)
- Ensured your robot arm is securely mounted and will not fall over when powered on.
    - You may be required to mount the robot arm on something that is at least 10cm tall to ensure the arm does not collide with the ground. *(This is a temporary requirement and more information will be provided in the future.)*
- Ensured the robot arm is not near any objects that could be damaged if the arm moves unexpectedly.
    - You should have at least 1 meter of clearance around the robot arm.

**It is strongly recommended to stay close to the robot** and be ready to power it off in case of any unexpected behavior.

See the [Configure MKS Motors and Hall Effect Sensors](https://discord.com/channels/1099629962618748958/1339645142172303440/1339645142172303440) thread on Discord for more information. 

This thread is also a work in progress. However, it contains some useful information on how to configure the motors and hall effect sensors.

## Overview

**ROS2 Arctos** is a **ROS2 package** designed for controlling the Arctos robot arm using **CAN-based motor drivers** and **MoveIt! for motion planning**. The project is structured into multiple packages, each handling a specific aspect of the robotic arm.

## Repository Structure

```
ros2_arctos/
│── arctos_bringup/            # Launch and runtime management
│── arctos_description/        # URDF and robot model files
│── arctos_hardware_interface/ # ROS2 control hardware abstraction
│── arctos_motor_driver/       # CAN motor driver implementation
│── arctos_moveit_base_xyz/    # MoveIt! base motion with X, Y and Z
│── arctos_moveit_config/      # MoveIt! motion planning configurations
│── scripts/                   # Utility scripts
│── assets/                    # Images and other assets
│── LICENSE                    # Project license
│── README.md                  # Project documentation
```

## Installation

### Requirements

Ensure you have **Ubuntu Jammy (22.04)** installed before proceeding.

It is recommended to follow the official installation guides for ROS2 and MoveIt:

- [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
    - Install `ros-humble-desktop` for the full desktop installation.
    - Install `ros-dev-tools` for the development tools.
- [MoveIt! 2 Installation](https://moveit.ai/install-moveit2/binary/)

### Setting Up the Workspace

This section will guide you through setting up the ROS2 workspace and installing the remaining dependencies.
Please note, this assumes **you have already installed ROS2 Humble**.

First, install the required dependencies:
- `can-utils` for the CAN utilities.
- `python3-rosdep` for the easily installing dependencies.
- `ros-humble-can-msgs` for the CAN messages.
- `ros-humble-ros2-control` for the ROS2 control packages.

```bash
sudo apt install can-utils python3-rosdep ros-humble-can-msgs ros-humble-ros2-control -y
```

Install the Python dependencies:

```bash
pip3 install python-can ruamel.yaml rich keyboard -y
```

Create a ROS2 workspace and clone the ROS2 Arctos repository inside the `src/` directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Arctos-Robotics/ros2_arctos.git
```

**Note**: If you are on a different branch, you need to checkout the branch you want to use.

```bash
cd ros2_arctos
git checkout <branch_name>
```

Then navigate to the workspace root:
```bash
cd ~/ros2_ws
```

### Building the Workspace

Before building the workspace, source your ROS2 installation:

**Note**:*It is recommended to add this to your `.bashrc` file*.

```bash
source /opt/ros/humble/setup.bash
```

Initialize `rosdep` and install the dependencies:

```bash
sudo rosdep init
rosdep update
```

Install the dependencies:

```bash
rosdep install --from-paths src -y --ignore-src
```

**Note**: You may encounter an error with the package `ros-humble-warehouse-ros-mongo`. You can ignore this package for now.

Build the workspace using `colcon`:

```bash
cd ros2_arctos
colcon build --symlink-install
source install/setup.bash
```

You should now have the workspace built and ready to use.

### Getting Started

#### Setup CAN Interface

For this section, we will temporarily move to the ros2_arctos directory in the src folder.
*In the future, we will handle this in a more user-friendly way.*

```bash
pushd src/ros2_arctos
```

Make the setup script executable:

```bash
chmod +x scripts/setup_canable.sh
```

To setup the CAN interface, run the script `setup_canable.sh`:

```bash
sudo ./scripts/setup_canable.sh
```

#### Configure the joint limits and zero positions (First time only)

**This only supports the joints XYZ at the moment.**

To configure the joint limits and zero positions, run the script `set_zero_position.py`.

**Note**: You need to run this script as root.

```bash
sudo -E python3 scripts/set_zero_position.py --init
```

Follow the instructions on the screen to set the zero positions and joint limits.

#### Set the zero positions (Every time you power the robot)

If this is the first time you are setting the zero positions, see the previous step and ignore this one.
You only need to run this step if you have already set the zero positions and you have powered off the robot.

To set the zero positions, run the script `set_zero_position.py`.

Note: You need to run this script as root.

```bash
sudo -E python3 scripts/set_zero_position.py --set-zero
```

Lastly, we will pop back to the workspace root and rebuild the workspace.

This is necessary to ensure the limits and zero positions are correctly set and reflected in the workspace.

Run the following command to pop back to the workspace root:
```bash
popd
```

Rebuild the workspace:

```bash
colcon build --symlink-install
```

#### Launch the robot

To launch the robot, run the launch file `arctos_bringup.launch.py`:

```bash
ros2 launch arctos_bringup arctos_bringup.launch.py
```

## Individual Package READMEs

Each package has its own **README.md** with more details:

- [arctos\_bringup](arctos_bringup/README.md)
- [arctos\_description](arctos_description/README.md)
- [arctos\_hardware\_interface](arctos_hardware_interface/README.md)
- [arctos\_motor\_driver](arctos_motor_driver/README.md)
- [arctos\_moveit\_base\_xyz](arctos_moveit_base_xyz/README.md)
- [arctos\_moveit\_config](arctos_moveit_config/README.md)

## Contributing

Please follow our [Contributing Guidelines](CONTRIBUTING.md) before making any changes to the project.

We also expect all contributors to adhere to our [Code of Conduct](CODE_OF_CONDUCT.md).

## License

This project is licensed under the [Apache License](LICENSE).

## Contact

For questions or contributions, use **GitHub Issues** or join our **[Discord Community](YOUR_DISCORD_INVITE_LINK)**.
