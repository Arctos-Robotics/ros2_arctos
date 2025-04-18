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
buffer = [0, 0, 0, 0, 0, 0]
newValue = False

def handle_button_release(button, buffer):
    if button == 0:
        print("Button A released")
        buffer[5] = 0
    elif button == 1:
        print("Button B released")
        buffer[4] = 0
    elif button == 3:
        print("Button X released")
        buffer[4] = 0
    elif button == 4:
        print("Button Y released")
        buffer[5] = 0
    elif button == 6:
        print("Left bumper released")
    elif button == 7:
        print("Right bumper released")
    elif button == 10:
        print("Back button released")
    elif button == 11:
        print("Start button released")
    elif button == 13:
        print("Left stick button released")
    elif button == 14:
        print("Right stick button released")
    elif button == 8:
        print("Left trigger (L2) released")
    elif button == 9:
        print("Right trigger (R2) released")
    else: 
        print("Helloooooo")
    
def handle_axis_motion(axis, value, buffer):
    if axis == 0:  # X-axis of the left stick
        print(f"Left stick X-axis moved to {value}")
        if value >= -1.1 and value < -0.9:
            buffer[0] = -1
        elif value > 0.9 and value <=1.1:
            buffer[0] = 1
        elif value >= -0.2 and value <= 0.2: buffer[0] = 0
    elif axis == 1:  # Y-axis of the left stick
        print(f"Left stick Y-axis moved to {value}")
        if value >= -1.1 and value < -0.9:
            buffer[1] = -1
        elif value > 0.9 and value <=1.1:
            buffer[1] = 1
        elif value >= -0.2 and value <= 0.2: buffer[1] = 0
    elif axis == 2:  # X-axis of the right stick
        print(f"Right stick X-axis moved to {value}")
        if value >= -1.1 and value < -0.9:
            buffer[3] = -1
        elif value > 0.9 and value <=1.1:
            buffer[3] = 1
        elif value >= -0.2 and value <= 0.2: buffer[3] = 0
    elif axis == 3:  # Y-axis of the right stick
        print(f"Right stick Y-axis moved to {value}")
        if value >= -1.1 and value < -0.9:
            buffer[2] = -1
        elif value > 0.9 and value <=1.1:
            buffer[2] = 1
        elif value >= -0.2 and value <= 0.2: buffer[2] = 0
    elif axis == 5:  # Left trigger (L2)
        print(f"Left trigger (L2) value: {value}")
    elif axis == 4:  # Right trigger (R2)
        print(f"Right trigger (R2) value: {value}")
    elif axis == 6:  # D-pad X-axis
        handle_dpad_x(value)
    elif axis == 7:  # D-pad Y-axis
        handle_dpad_y(value)

def handle_button_press(button, buffer):
    global debounce
    if button == 0:
        print("Button A pressed")
        buffer[5] = -1
    elif button == 1:
        print("Button B pressed")
        buffer[4] = 1
    elif button == 3:
        print("Button X pressed")
        buffer[4] = -1
    elif button == 4:
        print("Button Y pressed")
        buffer[5] = 1
    elif button == 6:
        print("Left bumper pressed")
    elif button == 7:
        print("Right bumper pressed")
        buffer[5] = -1
    elif button == 10:
        print("Back button pressed")
    elif button == 11:
        print("Start button pressed")
    elif button == 13:
        print("Left stick button pressed")
    elif button == 14:
        print("Right stick button pressed")
    elif button == 8:
        print("Left trigger (L2) pressed")
        debounce = -1
    elif button == 9:
        print("Right trigger (R2) pressed")
    else: 
        print("Helloooooo")
    
def handle_dpad_x(value):
    global debounce
    if value == 1.0:
        print("D-pad right pressed")
    elif value == -1.0:
        print("D-pad left pressed")
    else:
        print("D-pad X-axis released")
        debounce = 0

def handle_dpad_y(value):
    if value == 1.0:
        print("D-pad down pressed")
    elif value == -1.0:
        print("D-pad up pressed")
    else:
        print("D-pad Y-axis released")

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
