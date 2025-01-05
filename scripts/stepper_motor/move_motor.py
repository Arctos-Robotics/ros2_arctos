import can
import time
import argparse
from pathlib import Path

def degrees_to_encoder_counts(degrees):
    """
    Convert degrees to encoder counts
    One full rotation (360°) = 16384 counts (0x4000)
    """
    counts_per_degree = 16384 / 360
    counts = int(degrees * counts_per_degree)
    return counts

def setup_can_logging(channel, interface, bitrate):
    bus = can.interface.Bus(channel=channel, interface=interface, bitrate=bitrate)
    log_file = Path('servo_movements.log')
    logger = can.Logger(log_file)
    return bus, logger

def move_to_position(bus, logger, degrees, can_id=0x01, speed_rpm=600, acc=2):
    position = degrees_to_encoder_counts(degrees)
    speed_hex = speed_rpm.to_bytes(2, 'big')
    pos_hex = position.to_bytes(3, 'big', signed=True)
    
    data = [
        0xF5,        # Position Mode 4 command
        speed_hex[0], # Speed high byte
        speed_hex[1], # Speed low byte
        acc,         # Acceleration
        pos_hex[0],  # Position high byte
        pos_hex[1],  # Position middle byte
        pos_hex[2],  # Position low byte
    ]
    
    crc = sum([can_id] + data) & 0xFF
    data.append(crc)
    
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=False
    )
    
    bus.send(msg)
    logger.on_message_received(msg)

    response = bus.recv(timeout=1.0)
    if response:
        logger.on_message_received(response)
    
    return msg

def read_encoder_position(bus, can_id=0x03):
    data = [0x30]
    crc = sum([can_id] + data) & 0xFF
    data.append(crc)
    
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=False
    )
    
    print(f"candump message {msg}")

    bus.send(msg)
    logger.on_message_received(msg)

    # response = bus.recv(timeout=1.0)
    # if response:
    #     logger.on_message_received(response)
    #     position = int.from_bytes(response.data[1:4], 'big', signed=True)
    #     return position
    # return None

def enable_motor(can_id=0x03):
    data = [0xF1]
    crc = sum([can_id] + data) & 0xFF
    data.append(crc)
    
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=False
    )
    
    # Print the message in candump format like 003#F1F6
    print(f"cansend can0 00{msg.arbitration_id}#{''.join([f'{byte:02X}' for byte in msg.data])}")
def main():
    # parser = argparse.ArgumentParser(description='Control servo motor movement')
    # parser.add_argument('angles', type=float, nargs='+', help='List of angles to move to')
    # parser.add_argument('--can-id', type=int, default=1, help='CAN ID of the servo (default: 1)')
    # parser.add_argument('--channel', type=str, default='can0', help='CAN channel (default: can0)')
    # parser.add_argument('--bitrate', type=int, default=500000, help='Bitrate (default: 500000)')
    # parser.add_argument('--interface', type=str, default='socketcan', help='Interface type (default: socketcan)')
    # parser.add_argument('--speed', type=int, default=600, help='Speed in RPM (default: 600)')
    # parser.add_argument('--acceleration', type=int, default=2, help='Acceleration (default: 2)')
    # parser.add_argument('--wait', type=float, default=3.0, help='Wait time between movements in seconds (default: 3.0)')
    
    # args = parser.parse_args()
    
    # bus, logger = setup_can_logging(args.channel, args.interface, args.bitrate)
    
    # try:
    #     print(f"Starting movement logging test for motor ID: {args.can_id}")
    #     print(f"Moving through angles: {args.angles}")
        
    #     for angle in args.angles:
    #         print(f"Moving motor {args.can_id} to {angle} degrees")
    #         msg = move_to_position(bus, logger, angle, 
    #                              can_id=args.can_id,
    #                              speed_rpm=args.speed,
    #                              acc=args.acceleration)
    #         print(f"Sent message: {msg}")
            
    #         start_time = time.time()
    #         while time.time() - start_time < args.wait:
    #             response = bus.recv(timeout=0.1)
    #             if response:
    #                 logger.on_message_received(response)
    #                 print(f"Received response: {response}")
            
    # except KeyboardInterrupt:
    #     print("\nLogging stopped by user")
    # finally:
    #     logger.stop()
    #     bus.shutdown()
    enable_motor(None, can_id=0x03)

if __name__ == "__main__":
    main()