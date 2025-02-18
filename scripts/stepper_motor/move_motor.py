import can
import time
import argparse
from pathlib import Path

def degrees_to_encoder_counts(degrees, gear_ratio=1.0):
    """
    Convert degrees to encoder counts, considering the gear ratio.
    One full rotation (360°) = 16384 counts (0x4000)
    """
    counts_per_degree = 16384 / 360
    counts = int(degrees * counts_per_degree * gear_ratio)
    return counts

def setup_can_logging(channel, interface, bitrate):
    bus = can.interface.Bus(channel=channel, interface=interface, bitrate=bitrate)
    log_file = Path('servo_movements.log')
    logger = can.Logger(log_file)
    return bus, logger

def move_to_position(bus, logger, degrees, can_id=0x01, speed_rpm=600, acc=2, gear_ratio=1.0):
    position = degrees_to_encoder_counts(degrees, gear_ratio)
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

# Example usage:
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Move motor to a specified position.")
    parser.add_argument("degrees", type=float, help="Target position in degrees.")
    parser.add_argument("--gear_ratio", type=float, default=1.0, help="Gear ratio.")
    parser.add_argument("--can_channel", type=str, default="can0", help="CAN channel.")
    parser.add_argument("--can_interface", type=str, default="socketcan", help="CAN interface.")
    parser.add_argument("--can_bitrate", type=int, default=500000, help="CAN bitrate.")
    parser.add_argument("--can_id", type=int, default=0x01, help="CAN ID.")
    parser.add_argument("--speed_rpm", type=int, default=600, help="Speed in RPM.")
    parser.add_argument("--acc", type=int, default=2, help="Acceleration.")
    
    args = parser.parse_args()
    
    bus, logger = setup_can_logging(args.can_channel, args.can_interface, args.can_bitrate)
    move_to_position(bus, logger, args.degrees, args.can_id, args.speed_rpm, args.acc, args.gear_ratio)