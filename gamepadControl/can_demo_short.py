import os
import time
from typing import List
import can
import keyboard
from queue import Queue
import pygame
from mks_api import *
from game_pad import *
from utils import *
import asyncio
import traceback
import netifaces as ni
import logging
from pathlib import Path
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils



# the exact encoded value read from encoder
rawAxisArr = [0, 0, 0, 0, 0, 0]
# specify the direction that motor at index i must not rotate further.
# direction can either be 0 (DEC) or 1 (INC). -1 indicate the motor is in the min and max range.
mustStoppedBuffer = [-1, -1, -1, -1, -1, -1]
# position buffer: each element contain a list of 6 joint value, to move robot to the specified Joint position.
positionQueue = Queue(maxsize=10)
# The status of goHome.
goHome = False
# The status of blocking process to move to a xyz position
positionMove = False
# counter the number of motors that has gone home.
motorRunCounter = 0
# go home step
goHomeStep = 2
# status of each motor. Busy indicate motor is working. Rotating only valid for mode F6 (SpeedMode), indicate motor is rolling towards a direction
motorBusy = { i : {"busy": False, "rotating": False, "timeWaitedAck": 0} for i in range(1, 7)}
# speed configration for each motor. the maximum speed should not greater than 1000.
speedConfig = [20, 150, 100, 80, 150, 150]
# accelaration config for each motor. faster the accelaration, the faster motor reaching its specified speed above. max acceleration is 254
accelerationConfig = [60, 60, 60, 60, 60, 60]
# a queue to hold the command that will be sent to Canable.
commandQueue = Queue(maxsize=20)
# Axis has changed since the last update
axisChangedCommon = False
# not used.
speed = 20
oldSpeed = speed


# -------------------------------------------------------------------- CONTROLLER GLOBAL VARIABLES--------------------------------------------------------------------
isStoppedBufferController = [True, True, True, True, True, True]


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
IPAddr = "localhost"

#initialize Joystick
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Name of joystick: {joystick.get_name()}")

# invert kinematic using ikpy (such wow)
myChain = ikpy.chain.Chain.from_urdf_file("arctos.urdf")


# -------------------------------------------CAN section -------------------------------------------------------

def cyclicRead():
    global commandQueue
    if (not commandQueue.full()):
        commandQueue.put(prepareCanMessage(X_MOTOR_ID + 1, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(Y_MOTOR_ID + 1, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(Z_MOTOR_ID + 1, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(A_MOTOR_ID + 1, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(B_MOTOR_ID + 1, prepareReadEncoderValue()))
        commandQueue.put(prepareCanMessage(C_MOTOR_ID + 1, prepareReadEncoderValue()))

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
        if msg.data[0] != 0x31:
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
    global rawAxisArr, motorRunCounter, goHome, axisChangedCommon, goHomeStep, positionMove
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
                        if motorRunCounter >= 1:
                            motorRunCounter -= 1
                            if goHome == True and motorRunCounter <= 0 and goHomeStep <= 0:
                                goHome = False
                            if positionMove == True and motorRunCounter <= 0:
                                positionMove = False
                        print("Run axis completed")
                    elif value == 3:
                        motorBusy[receivedMsg.arbitration_id]["busy"] = False
                        print("Stopped due to end limit")
                    # motorBusy[receivedMsg.arbitration_id]["timeWaitedAck"] = time.time()
                # read encoder command
                elif receivedCommand == 0x31:
                    allowPrint = False
                    if value != rawAxisArr[receivedMsg.arbitration_id-1]:
                        rawAxisArr[receivedMsg.arbitration_id-1] = value
                        axisChangedCommon = True
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
                if motorIndex == X_MOTOR_ID:
                    commandQueue.put(prepareCanMessage(motorIndex+1, prepareSpeedmodeCommand(run = False, direction = DON_T_CARE, speed = ZERO, acceleration = 10)))
                else:
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
        axisProceesedValue = rawToProcessedAxisValue(rawAxisArr)
        coordinate = getCoordinate(rawAxisArr)
        print(f"Status updated: {rawAxisArr}")
        print(f"Status updated processed: {axisProceesedValue}")
        print(f"Status updated coordinate: {coordinate}")
        axisChangedCommon = False

def cyclicSafety():
    """
    Emergency stop the motor if it gone out of range.
    #TODO: change the value of mustStoppedMotor based on the real direction of the motor!
    """
    global mustStoppedBuffer
    motor_limits = [
        (MIN_XAXISMOTOR, MAX_XAXISMOTOR, AXIS_INVERTED[X_MOTOR_ID]),
        (MIN_YAXISMOTOR, MAX_YAXISMOTOR, AXIS_INVERTED[Y_MOTOR_ID]),
        (MIN_ZAXISMOTOR, MAX_ZAXISMOTOR, AXIS_INVERTED[Z_MOTOR_ID]),
        (MIN_AAXISMOTOR, MAX_AAXISMOTOR, AXIS_INVERTED[A_MOTOR_ID]),
        (MIN_BAXISMOTOR, MAX_BAXISMOTOR, AXIS_INVERTED[B_MOTOR_ID]),
        (MIN_CAXISMOTOR, MAX_CAXISMOTOR, AXIS_INVERTED[C_MOTOR_ID]),
    ]
    axisEncodedArr = rawToProcessedAxisValue(rawAxisArr)
    for i, (min_limit, max_limit, inverted) in enumerate(motor_limits):
        # Y motor and C and B motor has a very weird direction, why?
        if inverted:
            if axisEncodedArr[i] < min_limit:
                if mustStoppedBuffer[i] == -1:
                    mustStoppedBuffer[i] = DIRECTION_INC
                    rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
            elif axisEncodedArr[i] > max_limit:
                if mustStoppedBuffer[i] == -1:
                    mustStoppedBuffer[i] = DIRECTION_DEC
                    rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
            else:
                mustStoppedBuffer[i] = -1
        else:
            if axisEncodedArr[i] < min_limit:
                if mustStoppedBuffer[i] == -1:
                    mustStoppedBuffer[i] = DIRECTION_DEC
                    rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
            elif axisEncodedArr[i] > max_limit:
                if mustStoppedBuffer[i] == -1:
                    mustStoppedBuffer[i] = DIRECTION_INC
                    rotateMotor(motorIndex=i, direction=DON_T_CARE, stop=True)
            else:
                mustStoppedBuffer[i] = -1
        
def goHomeService():
    global motorRunCounter, goHome, goHomeStep, rawAxisArr
    # print(f'motorRunCounter = {motorRunCounter}')
    if goHome == True and motorRunCounter == 0:
        # 7 because we have 5 normal motors + 2 motors for 6th joint
        # TODO: change the number of counter here if we have less joints.
        processedAxisArr = rawToProcessedAxisValue(rawAxisArr)
        print(f"Status updated: {rawAxisArr}")
        print(f"Status updated processed: {processedAxisArr}")
        print("going home...")
        if goHomeStep == 2:
            print("go home step 2")
            print(f'C motor: {rawAxisArr[C_MOTOR_ID]} - {processedAxisArr[C_MOTOR_ID]} = {rawAxisArr[C_MOTOR_ID]-processedAxisArr[C_MOTOR_ID]}')
            print(f'B motor: {rawAxisArr[B_MOTOR_ID]} - {processedAxisArr[C_MOTOR_ID]} = {rawAxisArr[B_MOTOR_ID]-processedAxisArr[C_MOTOR_ID]}')
            motorRunCounter = 2
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
                                                        axis = -1 * int(rawAxisArr[C_MOTOR_ID]-processedAxisArr[C_MOTOR_ID]))))
            commandQueue.put(prepareCanMessage(B_MOTOR_ID+1, 
                            preparePositionModeAxisCommand(relative=False, 
                                                        speed = speedConfig[C_MOTOR_ID], 
                                                        acceleration = accelerationConfig[C_MOTOR_ID], 
                                                        axis = -1 * int(rawAxisArr[B_MOTOR_ID]-processedAxisArr[C_MOTOR_ID]))))
            goHomeStep = 1
        elif goHomeStep == 1:
            print("go home step 1")
            #rotate B axis
            motorRunCounter = NUM_OF_MOTOR
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


def moveToCoordinate(x, y, z):
    """
    1. convert angle to processed Axis (for B and C axis), limit it if processed Axis go out of range
    2. convert processed Axis to raw Axis (for B and C motor, which need control from Motor 5 and Motor 6)
    3. add the raw Axis to the positionQueue for robot to move.
    """
    global myChain
    print("======= coordinate -> raw =======")
    # 1. 
    targetPosition = [x,y,z]
    jointsAngles = myChain.inverse_kinematics(targetPosition)
    print("The angles of each joints are : ", jointsAngles)
    # 2.
    processedAxisArr = angleToProcessedAxis([jointsAngles[1], jointsAngles[2], jointsAngles[3], jointsAngles[4], jointsAngles[5], jointsAngles[6]])
    print(f"Processed Axis for Robot to reach {targetPosition} is {processedAxisArr}")
    # 3.
    rawAxis = processeedAxisValueToRaw(processedAxisArr)
    print(f"Raw Axis for Robot to reach {targetPosition} is {rawAxis}")

    return rawAxis

def getCoordinate(rawAxisArr):
    """
    rawAxisArr: array [6] of raw Axis Value
    1. get the processedAxisValue
    2. convert them to angle
    3. Compute forward kinematics for coordinate
    """
    global myChain
    print("======= raw -> coordinate =======")
    # 1.
    processedAxisArr = rawToProcessedAxisValue(rawAxisArr)
    print(f"Processed Axis of Robot is {processedAxisArr}")
    # 2.
    jointAngles = processedAxisToAngle(processedAxisArr)
    print(f"Angle of the Robot is {jointAngles}")
    # 3.
    realFrame = myChain.forward_kinematics([0, *jointAngles ,0, 0])
    print("Computed position vector : %s" % (realFrame[:3, 3]))

    return realFrame[:3, 3]

def setJointsValue(): 
    # With the axis mode, the motor run weird af.
    # motor 1 (X): request x, go to x
    # motor 2 (Y): request x, go to -x
    # motor 3 (Z): request x, go to x
    # motor 4 (A): request x, go to x
    # motor 5 (B): request x, go to -x
    # motor 6 (C): request x, go to -x
    global commandQueue, positionMove, motorRunCounter, positionQueue
        # moving is not completed
    if positionMove:
        return False
    # motors are free
    else:
        if positionQueue.empty():
            return False
        else:
            axisArray = positionQueue.get()
            for index, jointVal in enumerate(axisArray):
                if jointVal != None:
                    if index == Y_MOTOR_ID or index == B_MOTOR_ID or index == C_MOTOR_ID:
                        commandQueue.put(prepareCanMessage(index+1, preparePositionModeAxisCommand(relative=False, speed = speedConfig[index], acceleration = accelerationConfig[index], axis = -jointVal)))
                    else:
                        commandQueue.put(prepareCanMessage(index+1, preparePositionModeAxisCommand(relative=False, speed = speedConfig[index], acceleration = accelerationConfig[index], axis = jointVal)))
                    motorRunCounter += 1
                    positionMove = True
            print(f"Processing position {axisArray}")
            return True

# ------------------------------------------- /CAN section -------------------------------------------------------

async def main() -> None:

    async def processGamePad():
        global goHome, motorRunCounter, goHomeStep
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
        elif goHome == True or positionMove == True:
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
        global rawAxisArr, positionQueue
        # real bus
        # bus = can.interface.Bus(interface="slcan", channel="COM4", bitrate=500000)  
        # virtual bus
        bus = can.interface.Bus(interface="virtual", receive_own_messages=True)  

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

        # positionQueue.put([20000, None, None, None, None, None])
        # positionQueue.put([None, -100000, 20000, None, None, None])
        # positionQueue.put([None, 0, 0, None, None, None])
        # positionQueue.put([-20000, None, None, None, None, None])
        # positionQueue.put([None, -100000, 20000, None, None, None])
        # positionQueue.put([None, 0, 0, None, None, None])
        # positionQueue.put([0, None, 0, None, None, None])

        # joints = moveToCoordinate(    0 ,  -0.53031   , 0.2)
        # positionQueue.put(joints)

        while (True):
            setJointsValue()
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
    
    await asyncio.gather(updateRobot())


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
