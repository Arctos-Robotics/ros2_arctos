# reference: https://github.com/HMTCT/BKU_Thesis/tree/complete_dev/Driver

#!/usr/bin/env python3
import pygame
import time
import serial
import threading
# # Initialize pygame
# pygame.init()

# # serialPort = '/dev/ttyACM0'
# serialPort = 'COM3'
# serialBaudrate = 115200
# ack = False
# debounce = 0
# stopFlag = False
# # Initialize the joystick
# joystick_count = pygame.joystick.get_count()

# serialObject = serial.Serial(
#     port=serialPort, 
#     baudrate=serialBaudrate,
#     bytesize=serial.EIGHTBITS,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE, 
#     timeout=0.01
# ) 
# time.sleep(10)
# serialObject.write(bytes(str("!Initon#"), encoding='utf-8'))
# INitialize input values
DEBUG = True

buffer = [0, 0, 0, 0, 0, 0]
specialKey = [False, False, False, False]
newValue = False


def printDebugInput(data):
    if DEBUG:
        print(data)

def handle_button_release(button, buffer, specialKey):
    if button == 0:
        printDebugInput("Button X released")
        specialKey[button] = False
    elif button == 1:
        printDebugInput("Button O released")
        specialKey[button] = False
    elif button == 2:
        printDebugInput("Button ▢ released")
        specialKey[button] = False
    elif button == 3:
        printDebugInput("Button △ released")
        specialKey[button] = False
    elif button == 6:
        printDebugInput("Left bumper released")
    elif button == 7:
        printDebugInput("Right bumper released")
    elif button == 10:
        printDebugInput("Right trigger (R1) released")
    elif button == 11:
        printDebugInput("Up dpad button released")
        buffer[4] = 0
    elif button == 12:
        printDebugInput("Down dpad button released")
        buffer[4] = 0
    elif button == 13:
        printDebugInput("Left dpad button released")
        buffer[5] = 0
    elif button == 14:
        printDebugInput("Right dpad button released")
        buffer[5] = 0
    elif button == 8:
        printDebugInput("Left trigger (L2) released")
        debounce = -1
    elif button == 9:
        printDebugInput("Left trigger (L1) released")
    else: 
        printDebugInput(f"released {button}")
    
def handle_axis_motion(axis, value, buffer):
    if axis == 0:  # X-axis of the left stick
        printDebugInput(f"Left stick X-axis moved to {value}")
        #left
        if value >= -1.1 and value < -0.9:
            buffer[0] = 1
        #right
        elif value > 0.9 and value <=1.1:
            buffer[0] = -1
        elif value >= -0.2 and value <= 0.2: buffer[0] = 0
    elif axis == 1:  # Y-axis of the left stick
        printDebugInput(f"Left stick Y-axis moved to {value}")
        # up
        if value >= -1.1 and value < -0.9:
            buffer[1] = 1
        # down
        elif value > 0.9 and value <=1.1:
            buffer[1] = -1
        elif value >= -0.2 and value <= 0.2: buffer[1] = 0
    elif axis == 2:  # X-axis of the right stick
        printDebugInput(f"Right stick X-axis moved to {value}")
        
    elif axis == 3:  # Y-axis of the right stick
        printDebugInput(f"Right stick Y-axis moved to {value}")
        #left
        if value >= -1.1 and value < -0.9:
            buffer[3] = 1
        #right
        elif value > 0.9 and value <=1.1:
            buffer[3] = -1
        elif value >= -0.2 and value <= 0.2: buffer[3] = 0
    elif axis == 4:  # Left trigger (L2)
        printDebugInput(f"Left trigger (L2) value: {value}")
        # up
        if value >= -1.1 and value < -0.9:
            buffer[2] = -1
        # down
        elif value > 0.9 and value <=1.1:
            buffer[2] = 1
        elif value >= -0.2 and value <= 0.2: buffer[2] = 0
    elif axis == 5:  # Right trigger (R2)
        printDebugInput(f"Right trigger (R2) value: {value}")
    elif axis == 6:  # D-pad X-axis
        handle_dpad_x(value)
    elif axis == 7:  # D-pad Y-axis
        handle_dpad_y(value)

def handle_button_press(button, buffer, specialKey):
    global debounce
    if button == 0:
        printDebugInput("Button X pressed")
        specialKey[button] = True
    elif button == 1:
        printDebugInput("Button O pressed. Stop all motors")
        buffer = [0, 0, 0, 0, 0, 0]
        specialKey[button] = True
    elif button == 2:
        printDebugInput("Button ▢ pressed")
        specialKey[button] = True
    elif button == 3:
        printDebugInput("Button △ pressed")
        specialKey[button] = True
    elif button == 6:
        printDebugInput("Left bumper pressed")
    elif button == 7:
        printDebugInput("Right bumper pressed")
    elif button == 10:
        printDebugInput("Right trigger (R1) pressed")
    elif button == 11:
        printDebugInput("Up dpad button pressed")
        buffer[4] = 1
    elif button == 12:
        printDebugInput("Down dpad button pressed")
        buffer[4] = -1
    elif button == 13:
        printDebugInput("Left dpad button pressed")
        buffer[5] = 1
    elif button == 14:
        printDebugInput("Right dpad button pressed")
        buffer[5] = -1
    elif button == 8:
        printDebugInput("Left trigger (L2) pressed")
        debounce = -1
    elif button == 9:
        printDebugInput("Left trigger (L1) pressed")
    else: 
        printDebugInput(f"pressed {button}")
    
def handle_dpad_x(value):
    global debounce
    if value == 1.0:
        printDebugInput("D-pad right pressed")
        buffer[5] = -1
    elif value == -1.0:
        printDebugInput("D-pad left pressed")
        buffer[5] = 1
    else:
        printDebugInput(f"D-pad X-axis released for {value}")
        buffer[5] = 0

def handle_dpad_y(value):
    if value == 1.0:
        printDebugInput("D-pad up pressed")
        buffer[4] = 1
    elif value == -1.0:
        printDebugInput("D-pad down pressed")
        buffer[4] = -1
    else:
        printDebugInput(f"D-pad Y-axis released for {value}")
        buffer[4] = 0

# if joystick_count > 0:
#     joystick = pygame.joystick.Joystick(0)
#     joystick.init()

#     print(f"Joystick found: {joystick.get_name()}")
#     debounce = 0
#     while True:
#         # pygame.event.pump()
#         data = serialObject.readline().decode(encoding='utf-8')
#         if data == "ACK\r\n": 
#             print("acknowledage")
#             ack = True
#         elif data == "KCA\r\n": 
#             print("stopped")
#             ack = True
#         for event in pygame.event.get():
#             if event.type == pygame.JOYBUTTONDOWN:
#                 handle_button_press(event.button, buffer)
#             if event.type == pygame.JOYBUTTONUP:
#                 handle_button_release(event.button, buffer)
#             if event.type == pygame.JOYAXISMOTION:
#                 handle_axis_motion(event.axis, event.value, buffer)
#         newValue = False 
#         for b in buffer:
#             if b != 0:
#                 newValue = True
#                 stopFlag = False
#                 break
#         if debounce == 1:
#                 newValue = True
#                 stopFlag = False
         
#         if newValue == True and ack == True: 
#             if debounce == 1: 
#                 string = "!gohome#"
#                 debounce = 0 
#             else:                
#                 string = ''.join(str(x) for x in buffer)
#                 string = '!' + string + '#'
#             print('send: ', string)
#             serialObject.write(bytes(str(string), encoding='utf-8'))
#             ack = False
#         if newValue == False and stopFlag == False and ack == True:
#             stopFlag = True
#             string = "!000000#"
#             print('send: ', string)
#             serialObject.write(bytes(str(string), encoding='utf-8'))
#             ack = False
#     pygame.quit()

# else:
#     print("No joystick found.")
