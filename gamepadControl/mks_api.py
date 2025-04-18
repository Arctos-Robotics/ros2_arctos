# dictionary that conntains expected answer from encoder for each command. 
# for each answer, the [start, end] byte is specified.
# when processing, remember that the start byte is Most Significant Byte.
commandAnswer = {
    # relative position axis
    0xf4: [2, 2],
    # absolute position axis
    0xf5: [2, 2],
    # speed mode
    0xf6: [2, 2],
    # read encoder value
    0x31: [2, 7],
    # read motor real-time speed (RPM)
    0x32: [2, 3],
    # read go back to zero status
    0x3B: [2, 2]
}

def preparePositionModeAxisCommand(relative, speed, acceleration, axis) -> list[int]:
    """
    Prepare the data to:\n
    Move the motor to the specified axis with specified speed and acceleration.\n
    relative: True for relative, False for absolute\n
    speed: The speed of the motor (0-3000) (2 byte [2-3])\n
    acceleration: Acceleration/Deceleration of the motor (0-255) (1 byte [4])\n
    axis: The axis to move the motor to (-8388607 - +8388607) (3 byte [5-7])\n
    Need to append the arbitration ID and CRC with "prepareCanMessage" before sending.\n

    Relative response: \n
    F4 [status], with [status] = 0 (failed), 1 (start running), 2 (run completed), 3 (end stopper reached)\n

    Absolute response: \n
    F5 [status], with [status] = 0 (failed), 1 (start running), 2 (run completed), 3 (end stopper reached)\n
    """
    if relative == True:
        return [0xf4, speed >> 8, speed & 0xFF, acceleration & 0xFF, (axis >> 16) & 0xFF, (axis >> 8) & 0xFF, axis & 0xFF]
    else:
        return [0xf5, speed >> 8, speed & 0xFF, acceleration & 0xFF, (axis >> 16) & 0xFF, (axis >> 8) & 0xFF, axis & 0xFF]

def prepareSpeedmodeCommand(run: bool, direction:int, speed: int, acceleration: int) -> list[int]:
    """
    Prepare the data to spin the motor freely in direction, with speed and acceleration:\n
    run: True to run, False to stop\n
    Run the motor (True)
        direction: True for clockwise, False for counterclockwise\n
        speed: The speed of the motor (0-3000) (4 low bit of byte 2 and full byte 3)\n
        acceleration: Acceleration of the motor (0-255) (1 byte [4])\n
    Stop the motor (False)
        direction: False\n
        speed: 0x00\n
        acceleration: Deceleration of the motor (0-255) (1 byte [4])\n
    
    Response for run\n
    F6 [status], with [status] = 0 (failed), 1 (run completed)\n
    Response for stop\n
    F6 [status], with [status] = 0 (failed), 1 (stopping), 2 (stopped)\n
    """
    if run:
        return [0xf6, (0x00 | (direction << 7)) | (speed >> 8)  ,speed & 0xFF, acceleration & 0xFF]
    else:
        return [0xf6, 0x00, 0x00, acceleration & 0xFF]
    
def prepareReadEncoderValue() -> list[int]:
    """
    Prepare the data to read the encoder value:\n
    Response:\n
    31 [value], with [value] (6 bytes), signed int.\n
    """
    return [0x31]

def prepareReadMotorSpeed() -> list[int]:
    """
    Prepare the data to read the motor speed:\n
    Response:\n
    32 [value], with [value] (2 bytes), signed int.\n
    """
    return [0x32]

def prepareReadGoBackZero() -> list[int]:
    """
    Prepare the data to read the go back to zero:\n
    Response:\n
    3B [value], with [value] (1 byte): 0 (going back to zero), 1 (zero reached), 2 (failed)\n
    """
    return [0x3B]

def prepareSetCanID(id: int) -> list[int]:
    """
    Prepare the data to set the CAN ID:\n
    id: new ID for the Can slave\n
    Response:\n
    8B,  [value], with [value] (1 byte): 1 (success), 0 (failed)\n
    """
    return [0x8B, (id >> 8) & 0xFF, id & 0xFF]

def prepareGoHome() -> list[int]:
    """
    Prepare the data to go back to zero:\n
    Response:\n
    3B [value], with [value] (1 byte): 0 (go home failed), 1 (go home start), 2 (go home success)\n
    """
    return [0x91]

def prepareInitializeMotor(currentID: int, newID: int) -> list[int]:
    """
    Conduct a list of steps to initialize the motor:\n
    """

    # 2. Set working mode (SR_vFOC)
    setWorkingMode = [0x82, 5]
    # 3. Set Protection function (on)
    setProtection = [0x88, 1]
    # 4. Set subdivision interpolation to enable (255)
    setMplyer = [0x89, 1]
    # 5. Set home command
    # 90 [homeTrigger (low)] [homeDirection (CW)] [homeSpeed 00 60 (dec)] [Endlimit (enabled)]
    setHomeCommand = [90, 0, 0, 0, 60, 1]
    # 5. enable limit port mapping (only for 42D motor, which has CanID > 2)
    setLimitPortRemap = [0x9E, 1]
    # 99. Set the new ID
    setID = prepareSetCanID(newID)
    if (newID > 2):
        return [setWorkingMode, setProtection, setMplyer, setHomeCommand, setID]
    else:
        return [setWorkingMode, setProtection, setMplyer, setHomeCommand, setLimitPortRemap,setID]

