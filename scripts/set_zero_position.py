#!/usr/bin/env python3
import can
import keyboard
import math
import sys
import termios
import time
import tty
import argparse
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

ENCODER_RESOLUTION = 0x4000  # 16384 counts per revolution

# ARCTOS_YAML_PATH = "arctos_description/config/arctos_controller.yaml"
ARCTOS_YAML_PATH = "arctos_moveit_base_xyz/config/ros2_controllers.yaml"

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

def get_yaml_parameters(file_path, joint_name=None):
    """
    Get parameters for a motor joint from a YAML file.

    Args:
        file_path (str): Path to the YAML file.
        joint_name (str): The joint name (e.g., "X_joint").

    Returns:
        dict: Dictionary of parameters for the joint.
    """
    yaml = YAML()
    with open(file_path, 'r') as file:
        data = yaml.load(file)
    motors = data['arctos_hardware_interface']['ros__parameters']['motors']

    if joint_name is None:
        return motors
    elif joint_name in motors:
        return motors[joint_name]
    else:
        raise KeyError(f"Joint {joint_name} not found in the YAML file.")

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
            self.parameters = get_yaml_parameters(ARCTOS_YAML_PATH)
            self.joint_info = sorted(
                self.parameters.keys(), 
                key=lambda joint: self.parameters[joint]["motor_id"], 
                reverse=True  # Ensures highest motor ID goes first
            )
        console.print("[bold green]✓[/] Robot Controller initialized successfully")

    def get_joint_name_from_id(self, motor_id):
        """Get joint name from motor ID.
        
        Args:
            motor_id (int): The motor ID (1, 2, or 3).

        Returns:
            str: The joint name (e.g., "X_joint").
        """
        for joint_name, params in self.parameters.items():
            if params.get("motor_id") == motor_id:
                return joint_name
        return None
    
    def receive_message(self, message):
        """Callback for receiving CAN messages.
        
        Args:
            message (can.Message): The received CAN message.
        """
        self.can_resp = message

    def send_command(self, motor_id, command):
        """Send command and wait for response.
        
        Args:
            motor_id (int): The motor ID (1, 2, or 3).
            command (list): The command bytes to send.

        Returns:
            can.Message: The received CAN message.
        """
        self.can_resp = False
        self.notifier.add_listener(self.receive_message)
        
        msg = can.Message(
            arbitration_id=motor_id,
            data=bytearray(command) + bytes([get_CRC(motor_id, command)]),
            is_extended_id=False
        )
        self.bus.send(msg)
        
        ts = time.time()
        while not self.can_resp and time.time() - ts < TIMEOUT:
            time.sleep(0.01)
        
        self.notifier.remove_listener(self.receive_message)
        
        if not self.can_resp or not check_CRC(motor_id, self.can_resp.data) or motor_id != self.can_resp.arbitration_id:
            console.print("\n[bold red]ERROR[/] - No message received from Arctos arm")
            return None
        return self.can_resp

    def manual_move_joint(self, motor_id):
            """Manually move a joint using arrow keys.
            
            Args:
                motor_id (int): The motor ID (1, 2, or 3).

            Returns:
                None
            """
            # get the joint name in parameters from the joint index
            joint_name = self.get_joint_name_from_id(motor_id)
            degree_per_pulse = 1.8 / 16 / self.parameters[joint_name]["gear_ratio"]
            pulses = round(1 / degree_per_pulse)

            console.print(f"\n[bold blue]Manually moving {joint_name}...")
            console.print("[yellow]Use arrow keys to adjust. Press ESC to finish.[/]")
            
            # Print initial position line with fixed width
            print(f"\n{joint_name} position: 0.0000 rad    ", end='\r', flush=True)

            terminal_control = TerminalControl()
            terminal_control.disable_echo()

            def on_key_event(e):
                inverted = bool(self.parameters[joint_name].get("inverted"))

                if e.event_type == keyboard.KEY_DOWN:
                    if e.name == 'left':
                        direction = 0x80 if inverted else 0
                        self.relative_motion_by_pulses(motor_id, pulses, 100, 1, direction)
                        encoder_position = self.get_encoder_position(joint_name)
                        if encoder_position is not None:
                            # Use standard print with fixed width and padding
                            print(f"{joint_name} position: {encoder_position:8.4f} rad    ", end='\r', flush=True)
                    if e.name == 'right':
                        direction = 0 if inverted else 0x80
                        self.relative_motion_by_pulses(motor_id, pulses, 100, 1, direction)
                        encoder_position = self.get_encoder_position(joint_name)
                        if encoder_position is not None:
                            # Use standard print with fixed width and padding
                            print(f"{joint_name} position: {encoder_position:8.4f} rad    ", end='\r', flush=True)
                    elif e.name == 'esc':
                        encoder_position = self.get_encoder_position(joint_name)
                        # Clear the line before printing final position
                        print(" " * 50, end='\r')  # Clear any previous output
                        console.print(f"\n[blue]Zero position set at {encoder_position:.4f} radians for {joint_name}.[/]")
                        self.send_command(motor_id, [0x92])
                        update_yaml_parameters(ARCTOS_YAML_PATH, joint_name, {"zero_position": encoder_position})
                        self.stop_event.set()

            keyboard.hook(on_key_event, suppress=True)
            try:
                self.stop_event.wait()
            finally:
                keyboard.unhook_all()
                terminal_control.restore_echo()

    def move_to_absolute_position(self, motor_id, abs_position, speed=150, acceleration=2):
        """Move the motor to an absolute position.
        
        Args:
            motor_id (int): The motor ID (1, 2, or 3).
            abs_position (int): The absolute position in encoder counts.
            speed (int): The speed of the motor.
            acceleration (int): The acceleration of the motor.

        Returns:
            None
        """
        command = [
            0xF5,
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            acceleration,
            (abs_position >> 16) & 0xFF,
            (abs_position >> 8) & 0xFF,
            abs_position & 0xFF
        ]
        self.send_command(motor_id, command)

    def get_encoder_position(self, joint_name):
        """Get current encoder position and convert to radians.
        
        Args:
            joint_name (str): The joint name (e.g., "X_joint").

        Returns:
            float: The current joint position in radians.
        """
        motor_id = self.parameters[joint_name].get("motor_id")
        response = self.send_command(motor_id, [0x31])  # Command to read encoder position

        if response and hasattr(response, 'data') and len(response.data) >= 8:
            encoder_data = response.data[1:-1]
            try:
                position_deg = self.decode_int48(encoder_data)
                position_rad = (position_deg / self.parameters[joint_name]["gear_ratio"]) * (math.pi / 180)
                inverted = self.parameters[joint_name].get("inverted")
                if inverted:
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

    def relative_motion_by_pulses(self, motor_id, pulses, speed, acceleration, direction):
        """Execute relative motion command and return the response.
        
        Args:
            motor_id (int): The motor ID (1, 2, or 3).
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
        return self.send_command(motor_id, command)

    def find_home_position(self, motor_id):
        """Find home position using limit switch.
        
        Args:
            motor_id (int): The motor ID (1, 2, or 3).

        Returns:
            float: The home position in radians.
        """
        joint_name = self.get_joint_name_from_id(motor_id)
        degree_per_pulse = 1.8 / 16 / self.parameters[joint_name]["gear_ratio"]
        step_in_degree = 360
        pulses = round(step_in_degree / degree_per_pulse)
        inverted = self.parameters[joint_name].get("inverted")
        direction = 0x80 if inverted else 0
        
        console.print(f"\n[cyan]Finding home position for {joint_name}...")
        
        while True:
            response = self.relative_motion_by_pulses(motor_id, pulses, 50, 1, direction)
            if response and response.data[1] == 3:
                encoder_position = self.get_encoder_position(joint_name)
                if encoder_position is not None:
                    console.print(f"[green]{joint_name} home position found at: {encoder_position:.4f} radians[/]")
                    break
            time.sleep(0.5)

        return encoder_position

    def find_opposite_limit(self, motor_id):
        """Find the opposite limit switch by moving in the opposite direction.
        
        Args:
            motor_id (int): The motor ID (1, 2, or 3).

        Returns:
            float: The opposite limit position in radians.
        """
        joint_name = self.get_joint_name_from_id(motor_id)
        
        console.print(f"\n[cyan]Moving {joint_name} to zero position...")
        self.move_to_absolute_position(motor_id, 0)
        time.sleep(5)

        degree_per_pulse = 1.8 / 16 / self.parameters[joint_name]["gear_ratio"]
        step_in_degree = 360
        pulses = round(step_in_degree / degree_per_pulse)
        inverted = self.parameters[joint_name].get("inverted")
        direction = 0 if inverted else 0x80
        
        console.print(f"[cyan]Finding the opposite limit for {joint_name}...")
        while True:
            response = self.relative_motion_by_pulses(motor_id, pulses, 50, 1, direction)
            if response and response.data[1] == 3:
                encoder_position = self.get_encoder_position(joint_name)
                if encoder_position is not None:
                    console.print(f"[green]{joint_name} opposite limit found at: {encoder_position:.4f} radians[/]")
                    break
            time.sleep(0.5)
        
        return encoder_position

    def home_joint(self, motor_id):
        """Send command to home a joint and listen for the completion status.
        
        Args:
            motor_id (int): The motor ID (1, 2, or 3).

        Returns:
            None
        """
        console.print(f"\n[bold blue]Sending home command to motor {motor_id}...")
        
        homing_complete = Event()

        def response_listener(message):
            if message.arbitration_id == motor_id and message.data[1] == 2:
                console.print(f"[bold green]✓ Motor {motor_id} homed successfully!")
                homing_complete.set()

        self.notifier.add_listener(response_listener)

        try:
            self.send_command(motor_id, [0x91])
            with console.status(f"[bold cyan]Waiting for motor {motor_id} to complete homing..."):
                if not homing_complete.wait(timeout=300):
                    console.print(f"\n[bold red]ERROR: Homing for motor {motor_id} timed out!")
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
            
            for joint_name in self.joint_info:
                console.rule(f"[bold blue]Configuring {joint_name}")
                motor_id = self.parameters[joint_name].get("motor_id")
                gear_ratio = self.parameters[joint_name].get("gear_ratio")
                # Home the joint
                console.print(f"\n[green]Step 1: Homing {joint_name}...")
                self.home_joint(motor_id) # TODO: Fix this to prevent motor 3 from homing at the end because the joints are homed in reverse order and somehow motor 3 seems to be homed last because it's trying to get the index -1 of the joint_info list
                
                # Clear the stop event for the current joint
                self.stop_event.clear()
                
                # Manual adjustment
                console.print(f"\n[green]Step 2: Manual adjustment of {joint_name}...")
                self.manual_move_joint(motor_id)
                
                # Find limits
                console.print(f"\n[green]Step 3: Finding limits for {joint_name}...")
                home_encoder_position = self.find_home_position(motor_id)
                opposite_encoder_position = self.find_opposite_limit(motor_id)

                if motor_id != 3:
                    self.move_to_absolute_position(motor_id, 0)

                # Update YAML parameters
                update_yaml_parameters(ARCTOS_YAML_PATH, joint_name, {
                    "home_position": home_encoder_position,
                    "opposite_limit": opposite_encoder_position
                })
                
                console.print(f"[bold green]✓ {joint_name} configuration complete![/]\n")
            
            console.print("[bold green]✓ All joints configured successfully![/]")
            
        finally:
            terminal_control.restore_echo()

    def move_to_zero_position(self):
        """Move the joint to the zero position.
        
        Args:
            None

        Returns:
            None
        """
        for joint_name in self.joint_info:
            motor_id = self.parameters[joint_name].get("motor_id")
            gear_ratio = self.parameters[joint_name].get("gear_ratio")
            inverted = self.parameters[joint_name].get("inverted")
            zero_position = self.parameters[joint_name].get("zero_position")

            self.home_joint(motor_id)

            movement_complete = Event()

            def response_listener(message):
                if message.arbitration_id == motor_id:
                    if message.data[1] == 2:
                        console.print(f"[bold green]✓ Joint {motor_id} moved to zero position!")
                        movement_complete.set()
                    if message.data[1] == 3:
                        raise Exception(f"Endstop limit for motor ID {motor_id} triggered. Failed to move to zero position.")

            if zero_position is not None:
                self.notifier.add_listener(response_listener)

                zero_position = round(zero_position * gear_ratio * (180 / math.pi) * ENCODER_RESOLUTION / 360)

                if inverted:
                    zero_position = -zero_position

                try:
                    self.move_to_absolute_position(motor_id, zero_position)
                    with console.status(f"[bold cyan]Waiting for joint {joint_name} to move to zero_position..."):
                        if not movement_complete.wait(timeout=300):
                            console.print(f"\n[bold red]ERROR: Homing for joint {joint_name} timed out!")
                        if movement_complete.is_set():
                            time.sleep(1)
                            self.send_command(motor_id, [0x92])
                            console.print(f"[bold green]✓ Joint {motor_id} set to zero position!")

                finally:
                    self.notifier.remove_listener(response_listener)
        

    def shutdown(self):
        """Clean shutdown of CAN bus."""
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        console.print("[bold green]✓[/] Shutdown complete")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Arctos Robot Controller")
    parser.add_argument("--init", action="store_true", help="Configure joint limits and zero positions (Do this first)")
    parser.add_argument("--set-zero", action="store_true", help="Set zero position for all joints")
    parser.add_argument("--joint", type=str, nargs="+", help="Joint names to set zero position")

    args = parser.parse_args()

    try:
        controller = RobotController()

        if args.joint:
            controller.joint_info = [f"{joint}_joint" for joint in args.joint]

        if args.set_zero:
            controller.move_to_zero_position()
        elif args.init:
            console.print(Panel.fit(
                "[bold blue]Arctos Robot Controller[/]\nInitializing system...",
                border_style="blue"
            ))
            
            try:
                controller.configure_joints()
            except KeyboardInterrupt:
                console.print("\n[bold yellow]Interrupted by user. Shutting down...[/]")
        else:
            console.print("[bold yellow]No arguments provided. Use --init to configure the joints or --set-zero to set zero position.[/]")
            
    except Exception as e:
        console.print(f"\n[bold red]ERROR[/] - {e}")
    finally:
        controller.shutdown()
        console.print("[bold green]Session completed successfully![/]")
