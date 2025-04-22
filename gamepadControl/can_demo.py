import os
import time
from typing import List
import can
import keyboard
from queue import Queue
import pygame
from mks_api import *
from game_pad import *

pygame.init()
pygame.joystick.init()


joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Name of joystick: {joystick.get_name()}")

axisEncodedValue = [0, 0, 0, 0, 0, 0]
isStoppedBuffer = [False, False, False, False, False, False]

goHome = False
goHomeCounter = 0
isStopped = False
motorBusy = { i : {"busy": False, "rotating": False, "timeWaitedAck": 0} for i in range(1, 7)}
speedConfig = [100, 250, 250, 200, 20, 20]
accelarationConfig = [60, 60, 60, 60, 60, 60]
commandQueue = Queue(maxsize=20)

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
    global axisEncodedValue, goHomeCounter, goHome
    while buffReader.buffer.qsize() > 0:
        allowPrint = True
        receivedMsg = buffReader.get_message()
        if receivedMsg is not None:
            receivedCommand = receivedMsg.data[0]
            if receivedCommand in commandAnswer.keys():
                value = 0
                start, end = commandAnswer[receivedCommand]
                for i in range(start-1, end):
                    value |= (receivedMsg.data[i])
                    value = value << 8
                value = (value >> 8)
                # convert to negative value
                # check if the MSB is 1 (negative)
                if (value & (1 << (8 * (end-start+1) - 1))) != 0:
                    value = value - (1 << (8 * (end-start+1)))
                
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
                            if goHomeCounter > 1:
                                goHomeCounter -= 1
                            else:
                                goHome = False
                        print("Run axis completed")
                    elif value == 3:
                        motorBusy[receivedMsg.arbitration_id]["busy"] = False
                        print("Stopped due to end limit")
                    # motorBusy[receivedMsg.arbitration_id]["timeWaitedAck"] = time.time()
                elif receivedCommand == 0x31:
                    allowPrint = False
                    axisEncodedValue[receivedMsg.arbitration_id-1] = value
                if allowPrint:
                    print(f'Received: arbitration_id=0x{receivedMsg.arbitration_id:X}: {receivedCommand:X} {value}')
            else:
                received_data_bytes = ", ".join(
                [f"0x{byte:02X}" for byte in receivedMsg.data]
                )
                print(
                    f"Received: arbitration_id=0x{receivedMsg.arbitration_id:X}, data=[{received_data_bytes}], is_extended_id=False"
                )
        else:
            break

def checkAckTimeout():
    """
    Check the ack timeout, if the motor take too long to answer, release the lock "busy" and "rotating" to allow 
    control.
    """
    global motorBusy
    for id, motor in motorBusy.items():
        if motor["busy"] or motor["rotating"]:
            duration = time.time() - motor["timeWaitedAck"]
            if duration >= 5:
                motor["busy"] = False
                motor["rotating"] = False
                print(f"motorID:{id} ack timeout" )


def main() -> None:
    global isStopped, goHome, goHomeCounter
    """
    Main function to read CAN messages from a .txt file, send them through a CAN bus, and adjust speeds within packets.
    """
    # real bus
    bus = can.interface.Bus(interface="slcan", channel="COM3", bitrate=500000)  
    # virtual bus
    # bus = can.interface.Bus(interface="virtual", receive_own_messages=True)  

    print("Press arrow keys to call functions. Press ESC to exit.")

    buffReader = can.BufferedReader()
    notifier = can.Notifier(bus, [buffReader])

    delay_100ms = 0

    # initializeMotor(bus, currentID=0x01, newID=0x01)
    # initializeMotor(bus, currentID=0x02, newID=0x02)
    # initializeMotor(bus, currentID=0x03, newID=0x03)
    # initializeMotor(bus, currentID=0x04, newID=0x04)
    # initializeMotor(bus, currentID=0x05, newID=0x06)
    # initializeMotor(bus, currentID=0x05, newID=0x06)

    while (True):
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
            # 7 because we have 5 normal motors + 2 motors for 6th joint
            # TODO: change the number of counter here if we have less joints.
            goHomeCounter = 7
            print("going home...")
            # revert the 6th joint at axis X to 0:
            # - given 6th motor has ran to axis 0 + X -> X
            # - then 5th motor also ran to Y + X -> Z
            # - reset the 6th motor to by X - X -> 0
            # - rotate 5th motor by Z - X -> Y
            commandQueue.put(prepareCanMessage(5+1, preparePositionModeAxisCommand(relative=False, speed = speedConfig[5], acceleration = accelarationConfig[5], axis = axisEncodedValue[5]-axisEncodedValue[5])))
            commandQueue.put(prepareCanMessage(4+1, preparePositionModeAxisCommand(relative=False, speed = speedConfig[5], acceleration = accelarationConfig[5], axis = axisEncodedValue[4]-axisEncodedValue[5])))
            # revert the rest 5 motors back to 0
            for i in range(0, 5):
                commandQueue.put(prepareCanMessage(i+1, preparePositionModeAxisCommand(relative=False, speed = speedConfig[i], acceleration = accelarationConfig[i], axis = 0)))
        elif goHome == True:
            # do nothing, as we're going home
            print("going home...")
            pass
        else:
            for i in range(len(buffer)):
                if buffer[i] == -1:
                    if (not commandQueue.full()):
                        commandQueue.put(prepareCanMessage(i+1, prepareSpeedmodeCommand(run = True, direction = 0, speed = speedConfig[i], acceleration = accelarationConfig[i])))
                        # given 2 motor facing opposite direction, to make them "rotate in different direction", 
                        # means telling them to rotate in the same direction (relative to the motor)
                        if i == 5:
                            commandQueue.put(prepareCanMessage(i, prepareSpeedmodeCommand(run = True, direction = 0, speed = speedConfig[i], acceleration = accelarationConfig[i])))
                        isStoppedBuffer[i] = False
                elif buffer[i] == 1:
                    if (not commandQueue.full()):
                        commandQueue.put(prepareCanMessage(i+1, prepareSpeedmodeCommand(run = True, direction = 1, speed = speedConfig[i], acceleration = accelarationConfig[i])))
                        if i == 5:
                            commandQueue.put(prepareCanMessage(i, prepareSpeedmodeCommand(run = True, direction = 1, speed = speedConfig[i], acceleration = accelarationConfig[i])))
                        isStoppedBuffer[i] = False
                elif buffer[i] == 0:
                    if (isStoppedBuffer[i] == False and not commandQueue.full()):
                        commandQueue.put(prepareCanMessage(i+1, prepareSpeedmodeCommand(run = False, direction = 0, speed = 0, acceleration = 200)))
                        if i == 5:
                            commandQueue.put(prepareCanMessage(i, prepareSpeedmodeCommand(run = False, direction = 0, speed = 0, acceleration = 200)))
                        isStoppedBuffer[i] = True
                else:
                    raise ValueError("Wtf?")


        # a bunch of function that read the status of motors:
        if delay_100ms < 100:
            delay_100ms += 1
        else:
            cyclicRead()
            delay_100ms = 0

        
        processedMessage = processSendMessage(commandQueue, motorBusy)
        canSendMessage(bus, messages=processedMessage)
        time.sleep(0.01)
        processReceivedMessage(buffReader)
        checkAckTimeout()
        print(f"Status: {axisEncodedValue}")
        if keyboard.is_pressed("esc"):
            break
    notifier.stop()
    bus.shutdown()


if __name__ == "__main__":
    main()
