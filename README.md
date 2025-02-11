# ROS2 Arctos
![Discord](https://img.shields.io/discord/1099629962618748958?logo=discord&logoColor=%23FFFFFF&logoSize=auto)

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
│── LICENSE                    # Project license
│── README.md                  # Project documentation
```

## Installation

### Prerequisites

Ensure you have **Ubuntu Jammy (22.04)** installed before proceeding.

Follow the official installation guides:

- [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [MoveIt! 2 Installation](https://moveit.ai/install-moveit2/binary/)

### Setting Up the Workspace

Create a ROS2 workspace and clone the ROS2 Arctos repository inside the `src/` directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Arctos-Robotics/ros2_arctos.git
```

Then navigate to the workspace root:
```bash
cd ~/ros2_ws
```

### Building the Workspace

Before building the workspace, source your ROS2 installation:

```bash
source /opt/ros/humble/setup.bash
```

Then, build the workspace:

```bash
cd ros2_arctos
colcon build --symlink-install
source install/setup.bash
```

### Getting Started

Instructions to get started will be added soon. Stay tuned!

#### Setup CAN Interface

To setup the CAN interface, run the script `setup_canable.sh`:

```bash
sudo ./scripts/setup_canable.sh
```

#### Configure the joint limits and zero positions (First time only)

**This only supports the joints XYZ at the moment.**

To configure the joint limits and zero positions, run the script `set_zero_position.py`.

Note: You need to run this script as root.

You will also need ruamel.yaml, python-can and rich libraries. 
You can install them using pip:

```bash
pip3 install ruamel.yaml python-can rich
```

Then run the script:

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
