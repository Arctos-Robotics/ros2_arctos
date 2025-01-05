# README.md

This folder demonstrates how to control a stepper motor by specifying the degrees of rotation.

The folder data contains CAN logs of the stepper motor being controlled to rotate multiple degrees.

You can use these logs to replay the commands to the stepper motor:

```bash
python3 -m can.player -c can0 -i socketcan -b 500000 --ignore-timestamps -g 0.5 data/0x03_test_angles.log
```

**Please note:** The following examples are not related to the ROS2 package. They are just examples of how to control a stepper motor using CAN messages for the MKS SERVO42D and SERVO57D using a CANable device. 

You can use thes example scripts to better understand how the degree of rotation is calculated and how to control the stepper motor.

The script `move_motor.py` will move the stepper motor to different angles. It assumes that:
- The CAN interface is `can0`
- The CAN device is `socketcan`.
- The CAN device is connected to the stepper motor.
- The CAN device is using a baud rate of `500000`.
- The CAN device is using the can_id `0x03`.
- No gearbox is used. This means that the motor relies on its standard steps per revolution which is 200 steps per revolution or 1.8 degrees per step. (No gearbox ratio is used.)

**Windows**

```bash
python3 move_motor.py 0 90 -90 45 -45 0 --wait 0.5 --can-id 3 --channel COM5 --interface slcan
```

**Linux**

```bash
python3 move_motor.py 0 90 -90 45 -45 0 --wait 0.5 --can-id 3 --channel can0 --interface socketcan
```
