# arctos_hardware_interface

## Description

This package provides a ROS2 hardware interface of the arctos robot.

It uses the arctos_motor_driver package to control the motors and ros2_control to provide a hardware interface to ROS.

## Directories

The package is organized as follows:

```
arctos_hardware_interface
├── include
│   └── arctos_hardware_interface
│       ├── arctos_interface.hpp # Class to control the hardware interface
│       └── arctos_services.hpp  # Class to provide services to the hardware interface (Unused at the moment)
├── src
│   └── arctos_interface.cpp     # Implementation of the arctos_interface.hpp
│   └── arctos_services.cpp      # Implementation of the arctos_services.hpp (Unused at the moment)
├── launch                       # Launch files
```

## Note

This package is still in development and may have bugs.

## Disclaimer

We are not responsible for any damage caused by the use of this package. Use it at your own risk.