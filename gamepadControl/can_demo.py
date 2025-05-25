import os
import time
from typing import List
import can
import keyboard
from queue import Queue
import pygame
from mks_api import *
from game_pad import *
import asyncua
from asyncua import ua, uamethod, Server
import asyncio
import traceback
import netifaces as ni
import logging
import ast
from pathlib import Path

# --------------------------------------------------------------------CONSTANTS--------------------------------------------------------------------
DURATION = 0.01

NUM_OF_MOTOR = 4

X_MOTOR_ID = 0
Y_MOTOR_ID = 1
Z_MOTOR_ID = 2
A_MOTOR_ID = 3
B_MOTOR_ID = 4
C_MOTOR_ID = 5

MIN_XAXISMOTOR = -70000
MAX_XAXISMOTOR = 70000

MIN_YAXISMOTOR = -724748
MAX_YAXISMOTOR = 301154

MIN_ZAXISMOTOR = -416906
MAX_ZAXISMOTOR = 196355

MIN_AAXISMOTOR = -75000
MAX_AAXISMOTOR = 75000

MIN_BAXISMOTOR = -111000
MAX_BAXISMOTOR = 111000

MIN_CAXISMOTOR = -220000
MAX_CAXISMOTOR = 220000

DIRECTION_INC = 1
DIRECTION_DEC = 0

DON_T_CARE = 0
ZERO = 0
#TODO: replace direction 1 and 0 with this for clear code.

# -------------------------------------------------------------------- COMMON GLOBAL VARIABLES--------------------------------------------------------------------

# the exact encoded value read from encoder
axisEncodedValue = [0, 0, 0, 0, 0, 0]
# specify the direction that motor at index i must not rotate further.
# direction can either be 0 (DEC) or 1 (INC). -1 indicate the motor is in the min and max range.
mustStoppedBuffer = [-1, -1, -1, -1, -1, -1]
# The status of goHome.
goHome = False
# counter the number of motors that has gone home.
goHomeCounter = 0
# go home step
goHomeStep = 2
# status of each motor. Busy indicate motor is working. Rotating only valid for mode F6 (SpeedMode), indicate motor is rolling towards a direction
motorBusy = { i : {"busy": False, "rotating": False, "timeWaitedAck": 0} for i in range(1, 7)}
# speed configration for each motor. the maximum speed should not greater than 1000.
speedConfig = [40, 150, 100, 80, 150, 150]
# accelaration config for each motor. faster the accelaration, the faster motor reaching its specified speed above. max acceleration is 254
accelerationConfig = [60, 60, 60, 60, 60, 60]
# a queue to hold the command that will be sent to Canable.
commandQueue = Queue(maxsize=20)
# Axis has changed since the last update
axisChangedCommon = False
axisChangedOpcUA = False
# not used.
speed = 20
oldSpeed = speed


# -------------------------------------------------------------------- CONTROLLER GLOBAL VARIABLES--------------------------------------------------------------------
isStoppedBufferController = [True, True, True, True, True, True]

# -------------------------------------------------------------------- OPCUA GLOBAL VARIABLES--------------------------------------------------------------------
isStoppedBufferOpcUA = [True, True, True, True, True, True]


interfaces = ni.interfaces()
interface = None
if "ap0" in interfaces:
    interface = "ap0"
elif "wlan0" in interfaces:
    interface = "wlan0"
else:
    interface = "eth0"
_logger = logging.getLogger(__name__)
# IPAddr = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
IPAddr = "192.168.28.137"

#initialize Joystick
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Name of joystick: {joystick.get_name()}")


# -------------------------------------------CAN section -------------------------------------------------------

def cyclicRead():
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(0x01, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(0x02, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(0x03, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(0x04, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(0x05, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(0x06, prepareReadEncoderValue()))

def prepareCanMessage(arbitrationId: int, data: list[int]) -> can.Message:
    """
    Prepares a CAN message with the specified arbitration ID and data bytes. automatically calculates the CRC.
    """
    crc = sum(data) + arbitrationId & 0xFF
    data.append(crc)
    return can.Message(arbitration_id=arbitrationId, data=data, is_extended_id=False)

def processSendMessage(commandQueue: Queue[can.Message], motorBusy) -> List[can.Message]:
    """
    Processes a list of CAN commandQueue, checking if the motor is busy before sending them.

    Args:
        commandQueue: A list of `can.Message` objects to be sent.
        motorBusy: A list of dictionaries indicating the busy status of each motor.

    Note:
        This function checks if the motor is busy before sending commandQueue and updates the motor's status accordingly.
    """
    _processedMesssage = []
    while not commandQueue.empty():
        msg = commandQueue.get()
        send = False
        # if command is controlling and speed > 0
        if (msg.data[0] in [0xf4, 0xf5, 0xf6]) and ((msg.data[1] | msg.data[2]) > 0):
            # check if the motor is busy, if yes, discard the command
            if motorBusy[msg.arbitration_id]["busy"] == True:
                print("-", end="")
                continue
            elif motorBusy[msg.arbitration_id]["rotating"] == True and msg.data[0] == 0xf6:
                motorBusy[msg.arbitration_id]["timeWaitedAck"] = time.time()
                print(".", end="")
                continue
            else:
                # send it.
                motorBusy[msg.arbitration_id]["busy"] = True
                motorBusy[msg.arbitration_id]["timeWaitedAck"] = time.time()
                if msg.data[0] == 0xf6:
                    motorBusy[msg.arbitration_id]["rotating"] = True
                send = True
        else:
            send = True
        if send:
            _processedMesssage.append(msg)
    return _processedMesssage
            

def canSendMessage(bus: can.interface.Bus, messages: list) -> None:
    """
    Sends a list of CAN messages through a specified CAN bus and waits for responses.

    Args:
        bus: The `can.interface.Bus` instance representing the CAN bus to send messages on.
        messages: A list of `can.Message` objects to be sent.

    Note:
        This function waits for responses from expected motors after sending messages
        and prints out the status of the sent and received messages.
    """
    if len(messages) == 0:
        return
    for msg in messages:
        bus.send(msg)
        data_bytes = ", ".join([f"0x{byte:02X}" for byte in msg.data])
        print(
            f"Sent: arbitration_id=0x{msg.arbitration_id:X}, data=[{data_bytes}], is_extended_id=False"
        )

def initializeMotor(bus: can.interface.Bus, currentID: int, newID: int) -> None:
    """
    Initializes the motor by sending a series of commands to set its ID, working mode, protection function,
    subdivision interpolation, and home command.

    Args:
        bus: The `can.interface.Bus` instance representing the CAN bus to send messages on.
        currentID: The current CAN ID of the motor.
        newID: The new CAN ID to set for the motor.

    Note:
        This function sends a series of commands to the motor and waits for responses.
    """
    messages = prepareInitializeMotor(currentID, newID)
    for i in range(len(messages)):
            canSendMessage(bus, [prepareCanMessage(currentID, messages[i])])

def processReceivedMessage(buffReader: can.BufferedReader) -> None:
    global axisEncodedValue, goHomeCounter, goHome, axisChangedCommon, axisChangedOpcUA, goHomeStep
    while buffReader.buffer.qsize() > 0:
        allowPrint = True
        receivedMsg = buffReader.get_message()
        if receivedMsg is not None:
            receivedCommand = receivedMsg.data[0]
            if receivedCommand in commandAnswer.keys():
                value = 0
                start, end = commandAnswer[receivedCommand]
                try:
                    for i in range(start-1, end):
                        value |= (receivedMsg.data[i])
                        value = value << 8
                    value = (value >> 8)
                    # convert to negative value
                    # check if the MSB is 1 (negative)
                    if (value & (1 << (8 * (end-start+1) - 1))) != 0:
                        value = value - (1 << (8 * (end-start+1)))
                except Exception:
                    value = -99
                
                if receivedCommand == 0xf6:
                    if value == 2:
                        motorBusy[receivedMsg.arbitration_id]["rotating"] = False
                    else:
                        motorBusy[receivedMsg.arbitration_id]["busy"] = False
                    # motorBusy[receivedMsg.arbitration_id]["timeWaitedAck"] = time.time()
                elif receivedCommand in [0xf4, 0xf5]:
                    if value == 2:
                        motorBusy[receivedMsg.arbitration_id]["busy"] = False
                        # counting number of joints gone home.
                        if goHome == True:
                            if goHomeCounter >= 1:
                                goHomeCounter -= 1
                                if goHomeCounter <= 0 and goHomeStep <= 0:
                                    goHome = False
                        print("Run axis completed")
                    elif value == 3:
                        motorBusy[receivedMsg.arbitration_id]["busy"] = False
                        print("Stopped due to end limit")
                    # motorBusy[receivedMsg.arbitration_id]["timeWaitedAck"] = time.time()
                # read encoder command
                elif receivedCommand == 0x31:
                    allowPrint = False
                    if value != axisEncodedValue[receivedMsg.arbitration_id-1]:
                        axisEncodedValue[receivedMsg.arbitration_id-1] = value
                        axisChangedCommon = True
                        axisChangedOpcUA = True
                if allowPrint:
                    print(f'Received: arbitration_id=0x{receivedMsg.arbitration_id:X}: {receivedCommand:X} {value}')
                    # received_data_bytes = ", ".join(
                    # [f"0x{byte:02X}" for byte in receivedMsg.data]
                    # )
                    # print(
                    #     f"Received: arbitration_id=0x{receivedMsg.arbitration_id:X}, data=[{received_data_bytes}], is_extended_id=False"
                    # )
                    pass
            else:
                received_data_bytes = ", ".join(
                [f"0x{byte:02X}" for byte in receivedMsg.data]
                )
                print(
                    f"Received: arbitration_id=0x{receivedMsg.arbitration_id:X}, data=[{received_data_bytes}], is_extended_id=False"
                )
        else:
            break

def rotateMotor(motorIndex: int, direction: bool, stop: bool):
        global mustStoppedBuffer, commandQueue, speedConfig, accelerationConfig, isStoppedBufferController
        if stop:
            if isStoppedBufferController[motorIndex] == False and not commandQueue.full():
                commandQueue.put(prepareCanMessage(motorIndex+1, prepareSpeedmodeCommand(run = False, direction = DON_T_CARE, speed = ZERO, acceleration = 240)))
                isStoppedBufferController[motorIndex] = True
                # stop the 5th motor too if this motorIndex is 5 (motor 6)
                if motorIndex == B_MOTOR_ID and isStoppedBufferController[C_MOTOR_ID] == False:
                    commandQueue.put(prepareCanMessage(C_MOTOR_ID+1, prepareSpeedmodeCommand(run = False, direction = DON_T_CARE, speed = ZERO, acceleration = 240)))
                    isStoppedBufferController[C_MOTOR_ID] = True
                elif motorIndex == C_MOTOR_ID and isStoppedBufferController[B_MOTOR_ID] == False:
                    commandQueue.put(prepareCanMessage(B_MOTOR_ID+1, prepareSpeedmodeCommand(run = False, direction = DON_T_CARE, speed = ZERO, acceleration = 240)))
                    isStoppedBufferController[B_MOTOR_ID] = True
        # if the direction that the motor is spinning is not blocked by mustStoppedBuffer[motorIndex], then allow rotate
        elif (not commandQueue.full() and mustStoppedBuffer[motorIndex] != direction):
            commandQueue.put(prepareCanMessage(motorIndex+1, prepareSpeedmodeCommand(run = True, direction = direction, speed = speedConfig[motorIndex], acceleration = accelerationConfig[motorIndex])))
            isStoppedBufferController[motorIndex] = False
            # given 2 motor facing opposite direction, to make them "rotate in different direction", 
            # means telling them to rotate in the same direction (relative to the motor)
            if motorIndex == B_MOTOR_ID:
                commandQueue.put(prepareCanMessage(C_MOTOR_ID+1, prepareSpeedmodeCommand(run = True, direction = not direction, speed = speedConfig[motorIndex], acceleration = accelerationConfig[motorIndex])))
                isStoppedBufferController[C_MOTOR_ID] = False
            elif motorIndex == C_MOTOR_ID:
                commandQueue.put(prepareCanMessage(B_MOTOR_ID+1, prepareSpeedmodeCommand(run = True, direction = direction, speed = speedConfig[motorIndex], acceleration = accelerationConfig[motorIndex])))
                isStoppedBufferController[B_MOTOR_ID] = False
        else:
            print(f"Controller: Motor {motorIndex} go out of range! or queue full")

def getProcessedAxisValue(axisEncodedArr: list):
    processedAxis = [0, 0, 0, 0, 0, 0]
    for i in range(len(processedAxis)):
        if i == B_MOTOR_ID:
            processedAxis[i] = (axisEncodedArr[B_MOTOR_ID] - axisEncodedArr[C_MOTOR_ID])/2
        elif i == C_MOTOR_ID:
            processedAxis[i] = (processedAxis[B_MOTOR_ID] + axisEncodedArr[C_MOTOR_ID])
        else:
            processedAxis[i] = axisEncodedArr[i]
    return processedAxis

def cylicCheck():
    """
    Check the ack timeout, if the motor take too long to answer, release the lock "busy" and "rotating" to allow 
    control.
    """
    global motorBusy, axisChangedCommon
    for id, motor in motorBusy.items():
        if motor["busy"] or motor["rotating"]:
            duration = time.time() - motor["timeWaitedAck"]
            if duration >= 5:
                motor["busy"] = False
                motor["rotating"] = False
                print(f"motorID:{id} ack timeout" )
    if axisChangedCommon:
        axisProceesedValue = getProcessedAxisValue(axisEncodedValue)
        print(f"Status updated: {axisEncodedValue}")
        print(f"Status updated processed: {axisProceesedValue}")
        axisChangedCommon = False

def cyclicSafety():
    """
    Emergency stop the motor if it gone out of range.
    #TODO: change the value of mustStoppedMotor based on the real direction of the motor!
    """
    global mustStoppedBuffer
    motor_limits = [
        (MIN_XAXISMOTOR, MAX_XAXISMOTOR),
        (MIN_YAXISMOTOR, MAX_YAXISMOTOR),
        (MIN_ZAXISMOTOR, MAX_ZAXISMOTOR),
        (MIN_AAXISMOTOR, MAX_AAXISMOTOR),
        (MIN_BAXISMOTOR, MAX_BAXISMOTOR),
        (MIN_CAXISMOTOR, MAX_CAXISMOTOR),
    ]
    axisEncodedArr = getProcessedAxisValue(axisEncodedValue)
    for i, (min_limit, max_limit) in enumerate(motor_limits):
        # Y motor and C motor has a very weird direction, why?
        if i == Y_MOTOR_ID or i == B_MOTOR_ID or i == C_MOTOR_ID:
            if axisEncodedArr[i] < min_limit:
                if mustStoppedBuffer[i] == -1:
                    mustStoppedBuffer[i] = DIRECTION_INC
                    rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
                    rotateMotorOpcUA(motorIndex=i, direction=DON_T_CARE, stop=True)
            elif axisEncodedArr[i] > max_limit:
                if mustStoppedBuffer[i] == -1:
                    mustStoppedBuffer[i] = DIRECTION_DEC
                    rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
                    rotateMotorOpcUA(motorIndex=i, direction=DON_T_CARE, stop=True)
            else:
                mustStoppedBuffer[i] = -1
        else:
            if axisEncodedArr[i] < min_limit:
                if mustStoppedBuffer[i] == -1:
                    mustStoppedBuffer[i] = DIRECTION_DEC
                    rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
                    rotateMotorOpcUA(motorIndex=i, direction=DON_T_CARE, stop=True)
            elif axisEncodedArr[i] > max_limit:
                if mustStoppedBuffer[i] == -1:
                    mustStoppedBuffer[i] = DIRECTION_INC
                    rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
                    rotateMotorOpcUA(motorIndex=i, direction=DON_T_CARE, stop=True)
            else:
                mustStoppedBuffer[i] = -1
        
def goHomeService():
    global goHomeCounter, goHome, goHomeStep, axisEncodedValue
    # print(f'GohomeCounter = {goHomeCounter}')
    if goHome == True and goHomeCounter == 0:
        # 7 because we have 5 normal motors + 2 motors for 6th joint
        # TODO: change the number of counter here if we have less joints.
        processedAxisArr = getProcessedAxisValue(axisEncodedValue)
        print(f"Status updated: {axisEncodedValue}")
        print(f"Status updated processed: {processedAxisArr}")
        print("going home...")
        if goHomeStep == 2:
            print("go home step 2")
            print(f'C motor: {axisEncodedValue[C_MOTOR_ID]} - {processedAxisArr[C_MOTOR_ID]} = {axisEncodedValue[C_MOTOR_ID]-processedAxisArr[C_MOTOR_ID]}')
            print(f'B motor: {axisEncodedValue[B_MOTOR_ID]} - {processedAxisArr[C_MOTOR_ID]} = {axisEncodedValue[B_MOTOR_ID]-processedAxisArr[C_MOTOR_ID]}')
            goHomeCounter = 2
            # With the axis mode, the motor run weird af.
            # motor 1 (X): request x, go to x
            # motor 2 (Y): request x, go to -x
            # motor 3 (Z): request x, go to x
            # motor 4 (A): request x, go to x
            # motor 5 (B): request x, go to -x
            # motor 6 (C): request x, go to -x
            #rotate C axis
            commandQueue.put(prepareCanMessage(C_MOTOR_ID+1, 
                            preparePositionModeAxisCommand(relative=False, 
                                                        speed = speedConfig[C_MOTOR_ID], 
                                                        acceleration = accelerationConfig[C_MOTOR_ID], 
                                                        axis = -int(axisEncodedValue[C_MOTOR_ID]-processedAxisArr[C_MOTOR_ID]))))
            commandQueue.put(prepareCanMessage(B_MOTOR_ID+1, 
                            preparePositionModeAxisCommand(relative=False, 
                                                        speed = speedConfig[C_MOTOR_ID], 
                                                        acceleration = accelerationConfig[C_MOTOR_ID], 
                                                        axis = -int(axisEncodedValue[B_MOTOR_ID]-processedAxisArr[C_MOTOR_ID]))))
            # commandQueue.put(prepareCanMessage(Y_MOTOR_ID+1, 
            #                 preparePositionModeAxisCommand(relative=False, 
            #                                             speed = speedConfig[C_MOTOR_ID], 
            #                                             acceleration = accelerationConfig[C_MOTOR_ID], 
            #                                             axis = int(-10000))))
            # commandQueue.put(prepareCanMessage(B_MOTOR_ID+1, 
            #                 preparePositionModeAxisCommand(relative=False, 
            #                                             speed = speedConfig[C_MOTOR_ID], 
            #                                             acceleration = accelerationConfig[C_MOTOR_ID], 
            #                                             axis = int(-10000))))
            goHomeStep = 1
        elif goHomeStep == 1:
            print("go home step 1")
            #rotate B axis
            goHomeCounter = NUM_OF_MOTOR
            commandQueue.put(prepareCanMessage(C_MOTOR_ID+1, 
                            preparePositionModeAxisCommand(relative=False, 
                                                        speed = speedConfig[B_MOTOR_ID], 
                                                        acceleration = accelerationConfig[B_MOTOR_ID], 
                                                        axis = 0)))
            commandQueue.put(prepareCanMessage(B_MOTOR_ID+1, 
                            preparePositionModeAxisCommand(relative=False, 
                                                        speed = speedConfig[B_MOTOR_ID], 
                                                        acceleration = accelerationConfig[B_MOTOR_ID], 
                                                        axis = 0)))
            # revert the rest 4 motors back to 0
            for i in range(0, 4):
                commandQueue.put(prepareCanMessage(i+1, preparePositionModeAxisCommand(relative=False, speed = speedConfig[i], acceleration = accelerationConfig[i], axis = 0)))
            goHomeStep = 0

# ------------------------------------------- /CAN section -------------------------------------------------------

# ------------------------------------------- OPCUA section -------------------------------------------------------

xAxisMotor = 0
yAxisMotor = 0
zAxisMotor = 0
aAxisMotor = 0
bAxisMotor = 0
cAxisMotor = 0

methodDict = {}
subscriptionList = []

# -1: decrease angle, 0: stop, 1: increase angle
MOTOR_LOCK_AUTORUN = {
    "xAxis": 0,
    "yAxis": 0,
    "zAxis": 0,
    "aAxis": 0,
    "bAxis": 0,
    "cAxis": 0
}

def create_Ua_Argument(Name: str, Datatype: ua.NodeId, Description: str):
    """
    This function create an UA argument used to add more info to the method.
    """
    arg = ua.Argument()
    arg.Name = Name
    arg.DataType = Datatype
    arg.ValueRank = -1
    arg.ArrayDimensions = []
    arg.Description = ua.LocalizedText(Description)
    return arg


def type_conversion(list_of_string: list) -> list:
    """
    This function convert a string object into python type (if possible)
    """
    result = []
    for string in list_of_string:
        try:
            result.append(ast.literal_eval(string))
        except:
            result.append(string)
    return result

def rotateMotorOpcUA(motorIndex: int, direction: int, stop: bool):
        """
        Depend on the axis passed, it will handle the rest (motor 5 and motor 6)
        """
        global mustStoppedBuffer, commandQueue, speedConfig, accelerationConfig, isStoppedBufferOpcUA
        if stop:
            if isStoppedBufferOpcUA[motorIndex] == False and not commandQueue.full():
                commandQueue.put(prepareCanMessage(motorIndex+1, prepareSpeedmodeCommand(run = False, direction = DON_T_CARE, speed = ZERO, acceleration = 240)))
                isStoppedBufferOpcUA[motorIndex] = True
                # stop the 5th motor too if this motorIndex is 5 (motor 6)
                if motorIndex == B_MOTOR_ID and isStoppedBufferOpcUA[C_MOTOR_ID] == False:
                    commandQueue.put(prepareCanMessage(C_MOTOR_ID+1, prepareSpeedmodeCommand(run = False, direction = DON_T_CARE, speed = ZERO, acceleration = 240)))
                    isStoppedBufferOpcUA[C_MOTOR_ID] = True
                elif motorIndex == C_MOTOR_ID and isStoppedBufferOpcUA[B_MOTOR_ID] == False:
                    commandQueue.put(prepareCanMessage(B_MOTOR_ID+1, prepareSpeedmodeCommand(run = False, direction = DON_T_CARE, speed = ZERO, acceleration = 240)))
                    isStoppedBufferOpcUA[B_MOTOR_ID] = True
        # if the direction that the motor is spinning is not blocked by mustStoppedBuffer[motorIndex], then allow rotate
        elif (not commandQueue.full() and mustStoppedBuffer[motorIndex] != direction):
            commandQueue.put(prepareCanMessage(motorIndex+1, prepareSpeedmodeCommand(run = True, direction = direction, speed = speedConfig[motorIndex], acceleration = accelerationConfig[motorIndex])))
            isStoppedBufferOpcUA[motorIndex] = False
            # given 2 motor facing opposite direction, to make them "rotate in different direction", 
            # means telling them to rotate in the same direction (relative to the motor)
            if motorIndex == B_MOTOR_ID:
                commandQueue.put(prepareCanMessage(C_MOTOR_ID+1, prepareSpeedmodeCommand(run = True, direction = not direction, speed = speedConfig[motorIndex], acceleration = accelerationConfig[motorIndex])))
                isStoppedBufferOpcUA[C_MOTOR_ID] = False
            elif motorIndex == C_MOTOR_ID:
                commandQueue.put(prepareCanMessage(B_MOTOR_ID+1, prepareSpeedmodeCommand(run = True, direction = direction, speed = speedConfig[motorIndex], acceleration = accelerationConfig[motorIndex])))
                isStoppedBufferOpcUA[B_MOTOR_ID] = False
        else:
            print(f"Controller: Motor {motorIndex} go out of range! or queue full")

@uamethod
def moveMotor(parent, motor_name: str, angle_increase: bool, stop: bool):
    """
    Function that move entire axis.
    """
    global xAxisMotor, yAxisMotor, zAxisMotor, aAxisMotor, bAxisMotor, speed, JetMaxControlList
    # print(f'{motor_name}, {angle_increase}')
    motor_limits = [
        ("xAxis", MIN_XAXISMOTOR, MAX_XAXISMOTOR),
        ("yAxis", MIN_YAXISMOTOR, MAX_YAXISMOTOR),
        ("zAxis", MIN_ZAXISMOTOR, MAX_ZAXISMOTOR),
        ("aAxis", MIN_AAXISMOTOR, MAX_AAXISMOTOR),
        ("bAxis", MIN_BAXISMOTOR, MAX_BAXISMOTOR),
        ("cAxis", MIN_CAXISMOTOR, MAX_CAXISMOTOR),
    ]
    motorFound = False
    for i, (loop_motor_name, min_limit, max_limit) in enumerate(motor_limits):
        if (loop_motor_name == motor_name):
            motorFound = True
            # why yAxis has this weird ass rotation rule?
            if stop:
                rotateMotorOpcUA(motorIndex=i, direction = DON_T_CARE, stop = True)
            elif not angle_increase:
                rotateMotorOpcUA(motorIndex=i, direction = DIRECTION_DEC, stop = False)
            elif angle_increase:
                rotateMotorOpcUA(motorIndex=i, direction = DIRECTION_INC, stop = False)

    if not motorFound:
        print(f"NOTOK")
        return f"NOTOK"    
    else:
        print(f"{motor_name};{angle_increase};OK")
        return f"{motor_name};{angle_increase};OK"

@uamethod
def autoMotor(parent, motor_name: str, direction: int):
    global MOTOR_LOCK_AUTORUN
    if direction < -1 or direction > 1:
        return "NOTOK"
    if motor_name in MOTOR_LOCK_AUTORUN:
        MOTOR_LOCK_AUTORUN[motor_name] = direction
        return "OK"
    else:
        return "NOTOK"

    
@uamethod
def goHomeMethod(parent):
    global goHome, goHomeStep
    if not goHome:
        goHome = True
        goHomeStep = 2
        return "OK"
    return "NOTOK"

# ------------------------------------------- /OPCUA section ------------------------------------------------------- 

async def main() -> None:

    # ------------------------------------------- OPCUA MAIN section ------------------------------------------------------- 
    server = Server()

    await server.init()
    # endpoint: address to connect to
    server.set_endpoint(f"opc.tcp://{IPAddr}:4840")
    server.set_server_name("Robot Arm Server")

        # set all possible endpoint policies for client to connect through
    server.set_security_policy(
        [
            ua.SecurityPolicyType.NoSecurity,
            # ua.SecurityPolicyType.Basic256Sha256_SignAndEncrypt,
            # ua.SecurityPolicyType.Basic256Sha256_Sign,
        ]
    )
    # Certificate is required for opcua client from Unity to connect to?
    cert_base = Path(__file__).parent
    server_cert = Path(cert_base / "certificate-example.der")
    print(f'Oh hello there {server_cert}')

    await server.load_certificate(str(server_cert))

    # setup our namespace. An endpoint can have multiple namespace!
    uri = "http://robotarm.asyncua.io"
    idx = await server.register_namespace(uri)

     # add some nodes
    rootFolder = await server.nodes.objects.add_folder(idx, "Robot Arm")

    motor_folder = await rootFolder.add_folder(idx, "Motor")
    xAxisMotorNode = await motor_folder.add_variable(idx, "xAxis", 0, varianttype=ua.VariantType.Double)
    yAxisMotorNode = await motor_folder.add_variable(idx, "yAxis", 0, varianttype=ua.VariantType.Double)
    zAxisMotorNode = await motor_folder.add_variable(idx, "zAxis", 0, varianttype=ua.VariantType.Double)
    aAxisMotorNode = await motor_folder.add_variable(idx, "aAxis", 0, varianttype=ua.VariantType.Double)
    bAxisMotorNode = await motor_folder.add_variable(idx, "bAxis", 0, varianttype=ua.VariantType.Double)
    cAxisMotorNode = await motor_folder.add_variable(idx, "cAxis", 0, varianttype=ua.VariantType.Double)

    speedNode = await rootFolder.add_variable(idx, "speed", 5)

    imageFolder = await rootFolder.add_folder(idx, "Image")
    imageJetmaxNode = await imageFolder.add_variable(idx, "imageJetmax", b"a", datatype=ua.ObjectIds.ImageJPG)

    methodFolder = await rootFolder.add_folder(idx, "Method")

    controlMethod = await methodFolder.add_folder(idx, "Control Method")

    inargx = create_Ua_Argument(Name="Motor", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Motor name: xAxis, yAxis, zAxis, aAxis, bAxis, cAxis")
    inargy = create_Ua_Argument(Name="Increase angle", Datatype=ua.NodeId(ua.ObjectIds.Boolean),
                                Description="Increase the angle of motor")
    inargz = create_Ua_Argument(Name="STOP", Datatype=ua.NodeId(ua.ObjectIds.Boolean),
                                Description="STOP THE MOTOR")
    outarg = create_Ua_Argument(Name="Ack", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Acknowledgement")

    methodDict["moveMotor"] = await controlMethod.add_method(idx,
                                                             "moveMotor",
                                                             moveMotor,
                                                             [inargx, inargy, inargz],
                                                             [outarg])
    
    inargx = create_Ua_Argument(Name="Motor", Datatype=ua.NodeId(ua.ObjectIds.String),
                                Description="Motor name: xAxis, yAxis, zAxis, aAxis, bAxis, cAxis")
    inargy = create_Ua_Argument(Name="Direction", Datatype=ua.NodeId(ua.ObjectIds.Int16),
                                Description="Direction of motor, -1 to decrease, 0 to stop and 1 to increase")

    methodDict["autoMotor"] = await controlMethod.add_method(idx,
                                                             "autoMotor",
                                                             autoMotor,
                                                             [inargx, inargy],
                                                             [outarg])
    
    methodDict["goHome"] = await controlMethod.add_method(idx,
                                                          "goHome",
                                                          goHomeMethod,
                                                          [],
                                                          [outarg])
    
    async def autoRun():
        global isStoppedBufferOpcUA
        """
        This function will run in a loop which control the arm to
        move automatically. To do that, use call method [autoMotor]
        to enable the lock.
        :return: None
        """
        global MOTOR_LOCK_AUTORUN
        while True:
            try:
                motor_names = [
                    ("xAxis"),
                    ("yAxis"),
                    ("zAxis"),
                    ("aAxis"),
                    ("bAxis"),
                    ("cAxis"),
                ]
                for i, (motor_name) in enumerate(motor_names):
                    if MOTOR_LOCK_AUTORUN[motor_name] != 0:
                        moveMotor(idx, motor_name,
                                    ua.Variant(True, ua.VariantType.Boolean) if MOTOR_LOCK_AUTORUN[motor_name] == 1 
                                    else ua.Variant(False, ua.VariantType.Boolean), ua.Variant(False, ua.VariantType.Boolean))
                    else:
                        if isStoppedBufferOpcUA[i] == False:
                            # if this is bAxis and cAxis is rolling, don't stop bAxis
                            # if this is cAxis and bAxis is rolling, don't stop cAxis
                            if (motor_name == "bAxis" and MOTOR_LOCK_AUTORUN["cAxis"] != 0) or (motor_name == "cAxis" and MOTOR_LOCK_AUTORUN["bAxis"] != 0):
                                pass
                            else:
                                moveMotor(idx, motor_name,
                                            ua.Variant(True, ua.VariantType.Boolean), ua.Variant(True, ua.VariantType.Boolean))
                await asyncio.sleep(DURATION*2)
                # print("sleep 5 seconds")
            except Exception:
                traceback.print_exc()
            if keyboard.is_pressed("esc"):
                break
        print("Exit autoRun")

    async def updateOpcUA():
        global axisEncodedValue, axisChangedOpcUA
        """
        function to read Robot Arm angle data and update to asyncua object model (asyncua server)
        """
        global xAxisMotor, yAxisMotor, zAxisMotor, aAxisMotor, bAxisMotor, cAxisMotor
        while True:
            # print("I'm here")
            if not axisChangedOpcUA:
                pass
            else:
                axisEncodedArr = getProcessedAxisValue(axisEncodedValue)
                # print(f"Arr = {axisEncodedArr}")
                xAxisMotor = axisEncodedArr[0]
                yAxisMotor = axisEncodedArr[1]
                zAxisMotor = axisEncodedArr[2]
                aAxisMotor = axisEncodedArr[3]
                bAxisMotor = axisEncodedArr[4]
                cAxisMotor = axisEncodedArr[5]
                asyncioLoop = asyncio.get_running_loop()
                taskSet = set()
                try:
                    taskSet.add(asyncioLoop.create_task(xAxisMotorNode.write_value(float(xAxisMotor))))
                    taskSet.add(asyncioLoop.create_task(yAxisMotorNode.write_value(float(yAxisMotor))))
                    taskSet.add(asyncioLoop.create_task(zAxisMotorNode.write_value(float(zAxisMotor))))
                    taskSet.add(asyncioLoop.create_task(aAxisMotorNode.write_value(float(aAxisMotor))))
                    taskSet.add(asyncioLoop.create_task(bAxisMotorNode.write_value(float(bAxisMotor))))
                    taskSet.add(asyncioLoop.create_task(cAxisMotorNode.write_value(float(cAxisMotor))))
                    while len(taskSet) > 0:
                        await taskSet.pop()
                except Exception as e:
                    traceback.print_exc()
                axisChangedOpcUA = False
                # print(f'Status: {axisEncodedValue}')
            await asyncio.sleep(DURATION*2)
            if keyboard.is_pressed("esc"):
                break
        print("Exit updateOpcUA")

    async def serverStart():
        print("serveropc", threading.current_thread().getName())
        async with server:
            print(f'OPC server running at {server.endpoint[0]}://{server.endpoint[1]}')
            # subscrition chi chay duoc sau khi Server.start()
            # handler = SubHandler()
            # subscription = server.create_subscription(1, handler)
            # subscriptionList.extend([unityNode])
            # print(subscriptionList)
            # subscription.subscribe_data_change(subscriptionList)
            # asyncio.gather giup chay 2 ham async 1 cach "song song"
            # await asyncio.gather(autoRun(), updateVideoJetMax())
            await asyncio.gather(autoRun(), updateOpcUA())

        print("Exit server")



    # -------------------------------------------/OPCUA MAIN section ------------------------------------------------------- 
    async def processGamePad():
        global goHome, goHomeCounter, goHomeStep
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                handle_button_press(event.button, buffer, specialKey)
            if event.type == pygame.JOYBUTTONUP:
                handle_button_release(event.button, buffer, specialKey)
            if event.type == pygame.JOYAXISMOTION:
                handle_axis_motion(event.axis, event.value, buffer)

        if specialKey[0] == True and goHome == False:
            goHome = True
            goHomeStep = 2
        elif goHome == True:
            # do nothing, as we're going home
            # await asyncio.sleep(3)
            pass
        else:
            for i in range(len(buffer)):
                if buffer[i] == -1:
                    # print(f"Buffer[{i}] = {buffer[i]}, direction = 0")
                    rotateMotor(motorIndex=i, direction=DIRECTION_DEC, stop=False)
                elif buffer[i] == 1:
                    # print(f"Buffer[{i}] = {buffer[i]}, direction = 1")
                    rotateMotor(motorIndex=i, direction=DIRECTION_INC, stop=False)
                elif buffer[i] == 0:
                    # if joint C_MOTOR_ID is rolling (buffer[C_MOTOR_ID] != 0, then we don't stop joint B_MOTOR_ID (buffer[B_MOTOR_ID]) from rolling.)
                    # if joint B_MOTOR_ID is rolling (buffer[B_MOTOR_ID] != 0, then we don't stop joint C_MOTOR_ID (buffer[C_MOTOR_ID]) from rolling.)
                    if (i == B_MOTOR_ID and buffer[C_MOTOR_ID] != 0) or (i == C_MOTOR_ID and buffer[B_MOTOR_ID] != 0):
                        pass
                    else:
                        rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
                else:
                    raise ValueError("Wtf?")
            
    async def updateRobot():
        global axisEncodedValue
        # real bus
        bus = can.interface.Bus(interface="slcan", channel="COM3", bitrate=500000)  
        # virtual bus
        # bus = can.interface.Bus(interface="virtual", receive_own_messages=True)  

        print("Press arrow keys to call functions. Press ESC to exit.")

        buffReader = can.BufferedReader()
        notifier = can.Notifier(bus, [buffReader])

        delay_100ms = 0

        # initializeMotor(bus, currentID=0x01, newID=0x01) # correct axis
        # initializeMotor(bus, currentID=0x02, newID=0x02) # reverse axis ???
        # initializeMotor(bus, currentID=0x03, newID=0x03) # correct axis
        # initializeMotor(bus, currentID=0x04, newID=0x04) # correct axis
        # initializeMotor(bus, currentID=0x05, newID=0x05) # reverse axis ???
        # initializeMotor(bus, currentID=0x06, newID=0x06) # reverse axis ???

        while (True):
            await processGamePad()
            # a bunch of function that read the status of motors:
            if delay_100ms < 10:
                delay_100ms += 1
                # print(delay_100ms, end=",")
            else:
                cyclicRead()
                delay_100ms = 0

            goHomeService()
            processedMessage = processSendMessage(commandQueue, motorBusy)
            canSendMessage(bus, messages=processedMessage)
            await asyncio.sleep(DURATION)
            processReceivedMessage(buffReader)
            cyclicSafety()
            cylicCheck()
            if keyboard.is_pressed("esc"):
                break
        notifier.stop()
        bus.shutdown()
        print("Exit updateRobot")
    
    await asyncio.gather(updateRobot(), serverStart())
    # await asyncio.gather(updateRobot())


if __name__ == "__main__":
    try:
        logging.basicConfig(level=logging.ERROR)
        # asyncioLoop = asyncio.get_event_loop()
        print("main", threading.current_thread().name)
        asyncio.run(main())
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()
