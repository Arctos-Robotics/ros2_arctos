# arctos_motor_driver

## Description

This package provides a ROS interface to the [MKS SERVO42&57D](https://github.com/makerbase-motor/MKS-SERVO57D/tree/master) motor drivers. 
It was developed for the Arctos robot, but theoretically it should work with any project that uses these motor drivers.

It was built using the version 1.0.4 and 1.0.6 of the drivers.

## Directories

The package is organized as follows:

```
arctos_motor_driver
├── include
│   └── arctos_motor_driver
│       ├── can_protocol.hpp # Functions to send and receive CAN messages
│       ├── motor_driver.hpp # Class to control the motor drivers
│       └── motor_types.hpp  # Enumerations for the motor drivers (e.g motor modes, motor states, etc)
├── src
│   ├── can_protocol.cpp     # Implementation of the can_protocol.hpp
│   └── motor_driver.cpp     # Implementation of the motor_driver.hpp
├── test                     # Test files (Currently outdated)
```

## Note

This package **does not support all the features of the motor drivers**, only the ones that were necessary for the Arctos robot.

It is also important to note that **this package is still in development and may have bugs**.

## Disclaimer

We are not responsible for any damage caused by the use of this package. Use it at your own risk.