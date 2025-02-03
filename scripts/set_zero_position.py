#!/usr/bin/env python3
import can
import keyboard
import math
import sys
import termios
import time
import tty
from threading import Event

from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from ruamel.yaml import YAML

# Configuration
CAN_ID = "can0"
CAN_BITRATE = 500000
TIMEOUT = 1

# TODO: Update these values to add all joints
JOINT_NAMES = ["X_joint", "Y_joint", "Z_joint"]
GEAR_RATIOS = [13.5, 150., 150.]
INVERTED_DIRECTION = {"X_joint", "Y_joint", "Z_joint"}
ENCODER_RESOLUTION = 0x4000  # 16384 counts per revolution

ARCTOS_YAML_PATH = "arctos_description/config/arctos_controller.yaml"

console = Console(color_system="truecolor")

def update_yaml_parameters(file_path, joint_name, updates):
    """
    Updates multiple parameters for a motor joint in a YAML file.

    Args:
        file_path (str): Path to the YAML file.
        joint_name (str): The joint name (e.g., "X_joint").
        updates (dict): Dictionary of parameters to update with their new values.
    """
    yaml = YAML()
    yaml.preserve_quotes = True  # Preserve quotes and formatting

    # Read the YAML file
    with open(file_path, 'r') as file:
        data = yaml.load(file)

    # Navigate to the joint and update the parameters
    motors = data['arctos_hardware_interface']['ros__parameters']['motors']
    if joint_name in motors:
        for parameter, new_value in updates.items():
            motors[joint_name][parameter] = new_value
    else:
        raise KeyError(f"Joint {joint_name} not found in the YAML file.")

    # Write the updated data back to the file
    with open(file_path, 'w') as file:
        yaml.dump(data, file)

def create_status_table(joint_name, position=None, status=""):
    """Create a status table for the current joint operation.
    
    Args:
        joint_name (str): The joint name (e.g., "X_joint").
        position (float): The current joint position in radians.
        status (str): The current status message.

    Returns:
        Panel: A rich Panel object with the status table.
    """
    table = Table(box=box.ROUNDED, show_header=False, width=60)
    table.add_column("Property", style="cyan")
    table.add_column("Value", style="yellow")
    
    table.add_row("Joint", joint_name)
    table.add_row("Position", f"{position:.4f} rad" if position is not None else "Unknown")
    table.add_row("Status", status)
    
    return Panel(table, title="[bold blue]Joint Status", border_style="blue")

def get_CRC(id, data):
    """Calculate CRC by summing ID and data.
    
    Args:
        id (int): The CAN message ID.
        data (bytes): The data bytes of the message.

    Returns:
        int: The calculated CRC value.
    """
    return (id + sum(data)) & 0xFF

def check_CRC(id, data):
    """Verify CRC of received message.
    
    Args:
        id (int): The CAN message ID.
        data (bytes): The data bytes of the message.

    Returns:
        bool: True if the CRC is correct, False otherwise.
    """
    checksum = id + sum(data[:-1])
    return data[-1] == (checksum & 0xFF)

class TerminalControl:
    """Class to disable terminal echo and canonical mode."""
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def disable_echo(self):
        """ Disable echo and canonical mode."""
        tty.setcbreak(self.fd)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def restore_echo(self):
        """Restore terminal settings."""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

class RobotController:
    def __init__(self):
        with console.status("[bold green]Initializing Robot Controller..."):
            self.bus = can.Bus(interface='socketcan', channel=CAN_ID, bitrate=CAN_BITRATE)
            self.can_resp = False
            self.notifier = can.Notifier(self.bus, [])
            self.stop_event = Event()
            self.skip_zero = False
        console.print("[bold green]✓[/] Robot Controller initialized successfully")

    def receive_message(self, message):
        """Callback for receiving CAN messages.
        
        Args:
            message (can.Message): The received CAN message.
        """
        self.can_resp = message

    def send_command(self, joint, command):
        """Send command and wait for response.
        
        Args:
            joint (int): The joint number (1, 2, or 3).
            command (list): The command bytes to send.

        Returns:
            can.Message: The received CAN message.
        """
        self.can_resp = False
        self.notifier.add_listener(self.receive_message)
        
        msg = can.Message(
            arbitration_id=joint,
            data=bytearray(command) + bytes([get_CRC(joint, command)]),
            is_extended_id=False
        )
        self.bus.send(msg)
        
        ts = time.time()
        while not self.can_resp and time.time() - ts < TIMEOUT:
            time.sleep(0.01)
        
        self.notifier.remove_listener(self.receive_message)
        
        if not self.can_resp or not check_CRC(joint, self.can_resp.data) or joint != self.can_resp.arbitration_id:
            console.print("\n[bold red]ERROR[/] - No message received from Arctos arm")
            return None
        return self.can_resp

    def manual_move_joint(self, joint_index):
            """Manually move a joint using arrow keys.
            
            Args:
                joint_index (int): The joint index (0, 1, or 2).

            Returns:
                None
            """
            joint_name = JOINT_NAMES[joint_index]
            degree_per_pulse = 1.8 / 16 / GEAR_RATIOS[joint_index]
            pulses = round(1 / degree_per_pulse)

            console.print(f"\n[bold blue]Manually moving {joint_name}...")
            console.print("[yellow]Use arrow keys to adjust. Press ESC to finish.[/]")
            
            # Print initial position line with fixed width
            print(f"\n{joint_name} position: 0.0000 rad    ", end='\r', flush=True)

            terminal_control = TerminalControl()
            terminal_control.disable_echo()

            def on_key_event(e):
                if e.event_type == keyboard.KEY_DOWN:
                    if e.name == 'left':
                        direction = 0x80 if joint_name in INVERTED_DIRECTION else 0
                        self.relative_motion_by_pulses(joint_index + 1, pulses, 100, 1, direction)
                        encoder_position = self.get_encoder_position(joint_name)
                        if encoder_position is not None:
                            # Use standard print with fixed width and padding
                            print(f"{joint_name} position: {encoder_position:8.4f} rad    ", end='\r', flush=True)
                    if e.name == 'right':
                        direction = 0 if joint_name in INVERTED_DIRECTION else 0x80
                        self.relative_motion_by_pulses(joint_index + 1, pulses, 100, 1, direction)
                        encoder_position = self.get_encoder_position(joint_name)
                        if encoder_position is not None:
                            # Use standard print with fixed width and padding
                            print(f"{joint_name} position: {encoder_position:8.4f} rad    ", end='\r', flush=True)
                    elif e.name == 'esc':
                        encoder_position = self.get_encoder_position(joint_name)
                        # Clear the line before printing final position
                        print(" " * 50, end='\r')  # Clear any previous output
                        console.print(f"\n[blue]Zero position set at {encoder_position:.4f} radians for {joint_name}.[/]")
                        self.send_command(joint_index + 1, [0x92])
                        update_yaml_parameters(ARCTOS_YAML_PATH, joint_name, {"zero_position": encoder_position})
                        self.stop_event.set()

            keyboard.hook(on_key_event, suppress=True)
            try:
                self.stop_event.wait()
            finally:
                keyboard.unhook_all()
                terminal_control.restore_echo()

    def move_to_absolute_position(self, joint_index, abs_position, speed=200, acceleration=2):
        """Move the motor to an absolute position.
        
        Args:
            joint_index (int): The joint index (0, 1, or 2).
            abs_position (int): The absolute position in encoder counts.
            speed (int): The speed of the motor.
            acceleration (int): The acceleration of the motor.

        Returns:
            None
        """
        joint = joint_index + 1
        command = [
            0xF5,
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            acceleration,
            (abs_position >> 16) & 0xFF,
            (abs_position >> 8) & 0xFF,
            abs_position & 0xFF
        ]
        self.send_command(joint, command)

    def get_encoder_position(self, joint_name):
        """Get current encoder position and convert to radians.
        
        Args:
            joint_name (str): The joint name (e.g., "X_joint").

        Returns:
            float: The current joint position in radians.
        """
        joint_index = JOINT_NAMES.index(joint_name)
        response = self.send_command(joint_index + 1, [0x31])  # Command to read encoder position

        if response and hasattr(response, 'data') and len(response.data) >= 8:
            encoder_data = response.data[1:-1]
            try:
                position_deg = self.decode_int48(encoder_data)
                position_rad = (position_deg / GEAR_RATIOS[joint_index]) * (math.pi / 180)
                if joint_name in INVERTED_DIRECTION:
                    position_rad = -position_rad
                return position_rad
            except ValueError as e:
                console.print(f"\n[red]Error decoding encoder data for {joint_name}: {e}[/]")
        return None

    def decode_int48(self, data):
        """Decodes a 48-bit signed integer from a byte array.
        
        Args:
            data (bytes): The byte array containing the 48-bit integer.

        Returns:
            float: The decoded 48-bit integer as a floating-point value.
        """
        if len(data) != 6:
            raise ValueError("Data size must be 6 bytes for int48_t decoding")

        addition_value = (
            (data[0] << 40) |
            (data[1] << 32) |
            (data[2] << 24) |
            (data[3] << 16) |
            (data[4] << 8) |
            data[5]
        )

        if addition_value & (1 << 47):
            addition_value -= (1 << 48)

        motor_angle_deg = (addition_value * 360.0) / ENCODER_RESOLUTION
        return motor_angle_deg

    def relative_motion_by_pulses(self, joint, pulses, speed, acceleration, direction):
        """Execute relative motion command and return the response.
        
        Args:
            joint (int): The joint number (1, 2, or 3).
            pulses (int): The number of pulses to move.
            speed (int): The speed of the motor.
            acceleration (int): The acceleration of the motor.
            direction (int): The direction of motion (0 or 0x80).

        Returns:
            can.Message: The received CAN message.
        """
        command = [
            0xFD,
            direction + ((speed >> 8) & 0b1111),
            speed & 0xFF,
            acceleration,
            (pulses >> 16) & 0xFF,
            (pulses >> 8) & 0xFF,
            (pulses >> 0) & 0xFF,
        ]
        return self.send_command(joint, command)

    def find_home_position(self, joint_index):
        """Find home position using limit switch.
        
        Args:
            joint_index (int): The joint index (0, 1, or 2).

        Returns:
            float: The home position in radians.
        """
        joint_name = JOINT_NAMES[joint_index]
        degree_per_pulse = 1.8 / 16 / GEAR_RATIOS[joint_index]
        step_in_degree = 360
        pulses = round(step_in_degree / degree_per_pulse)
        direction = 0x80 if joint_name in INVERTED_DIRECTION else 0
        
        console.print(f"\n[cyan]Finding home position for {joint_name}...")
        
        while True:
            response = self.relative_motion_by_pulses(joint_index + 1, pulses, 50, 1, direction)
            if response and response.data[1] == 3:
                encoder_position = self.get_encoder_position(joint_name)
                if encoder_position is not None:
                    console.print(f"[green]{joint_name} home position found at: {encoder_position:.4f} radians[/]")
                    break
            time.sleep(0.5)

        return encoder_position

    def find_opposite_limit(self, joint_index):
        """Find the opposite limit switch by moving in the opposite direction.
        
        Args:
            joint_index (int): The joint index (0, 1, or 2).

        Returns:
            float: The opposite limit position in radians.
        """
        joint_name = JOINT_NAMES[joint_index]
        
        console.print(f"\n[cyan]Moving {joint_name} to zero position...")
        self.move_to_absolute_position(joint_index, 0)
        time.sleep(5)

        degree_per_pulse = 1.8 / 16 / GEAR_RATIOS[joint_index]
        step_in_degree = 360
        pulses = round(step_in_degree / degree_per_pulse)
        direction = 0 if joint_name in INVERTED_DIRECTION else 0x80
        
        console.print(f"[cyan]Finding the opposite limit for {joint_name}...")
        while True:
            response = self.relative_motion_by_pulses(joint_index + 1, pulses, 50, 1, direction)
            if response and response.data[1] == 3:
                encoder_position = self.get_encoder_position(joint_name)
                if encoder_position is not None:
                    console.print(f"[green]{joint_name} opposite limit found at: {encoder_position:.4f} radians[/]")
                    break
            time.sleep(0.5)
        
        self.move_to_absolute_position(joint_index, 0)
        return encoder_position

    def home_joint(self, joint):
        """Send command to home a joint and listen for the completion status.
        
        Args:
            joint (int): The joint number (1, 2, or 3).

        Returns:
            None
        """
        console.print(f"\n[bold blue]Sending home command to joint {joint}...")
        
        homing_complete = Event()

        def response_listener(message):
            if message.arbitration_id == joint and message.data[1] == 2:
                console.print(f"[bold green]✓ Joint {joint} homed successfully!")
                homing_complete.set()

        self.notifier.add_listener(response_listener)

        try:
            self.send_command(joint, [0x91])
            with console.status(f"[bold cyan]Waiting for joint {joint} to complete homing..."):
                if not homing_complete.wait(timeout=30):
                    console.print(f"\n[bold red]ERROR: Homing for joint {joint} timed out!")
        finally:
            self.notifier.remove_listener(response_listener)

    def configure_joints(self):
        """Configure all joints.
        
        This function will home each joint, allow manual adjustment, and find the limits.
        The results are then saved to the YAML file under the appropriate ros__parameters.
        """
        try:
            terminal_control = TerminalControl()
            terminal_control.disable_echo()
            
            console.print("\n[bold cyan]Starting joint configuration...[/]")
            
            for joint_index, joint_name in enumerate(JOINT_NAMES):
                console.rule(f"[bold blue]Configuring {joint_name}")
                
                # Home the joint
                console.print(f"\n[green]Step 1: Homing {joint_name}...")
                self.home_joint(joint_index + 1)
                
                # Clear the stop event for the current joint
                self.stop_event.clear()
                
                # Manual adjustment
                console.print(f"\n[green]Step 2: Manual adjustment of {joint_name}...")
                self.manual_move_joint(joint_index)
                
                # Find limits
                console.print(f"\n[green]Step 3: Finding limits for {joint_name}...")
                home_encoder_position = self.find_home_position(joint_index)
                opposite_encoder_position = self.find_opposite_limit(joint_index)

                # Update YAML parameters
                update_yaml_parameters(ARCTOS_YAML_PATH, joint_name, {
                    "home_position": home_encoder_position,
                    "opposite_limit": opposite_encoder_position
                })
                
                console.print(f"[bold green]✓ {joint_name} configuration complete![/]\n")
            
            console.print("[bold green]✓ All joints configured successfully![/]")
            
        finally:
            terminal_control.restore_echo()

    def shutdown(self):
        """Clean shutdown of CAN bus."""
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        console.print("[bold green]✓[/] Shutdown complete")

def main():
    console.print(Panel.fit(
        "[bold blue]Arctos Robot Controller[/]\nInitializing system...",
        border_style="blue"
    ))
    
    controller = RobotController()
    try:
        controller.configure_joints()
    except KeyboardInterrupt:
        console.print("\n[bold yellow]Interrupted by user. Shutting down...[/]")
    finally:
        controller.shutdown()
        console.print("[bold green]Session completed successfully![/]")

if __name__ == "__main__":
    main()