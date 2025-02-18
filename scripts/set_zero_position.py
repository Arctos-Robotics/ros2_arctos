#!/usr/bin/env python3
import os
import can
import keyboard
import math
import sys
import termios
import time
import tty
import argparse
from threading import Event, Thread

from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.live import Live

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
        file_path (str): Path to the YAML file
        joint_name (str): Joint name
        updates (dict): Dictionary of parameters to update

    Returns:
        None
    """
    yaml = YAML()
    yaml.preserve_quotes = True  # Preserve quotes and formatting

    with open(file_path, 'r') as file:
        data = yaml.load(file)

    motors = data['arctos_hardware_interface']['ros__parameters']['motors']
    if joint_name in motors:
        for parameter, new_value in updates.items():
            motors[joint_name][parameter] = new_value
    else:
        raise KeyError(f"Joint {joint_name} not found in the YAML file.")

    with open(file_path, 'w') as file:
        yaml.dump(data, file)

def get_yaml_parameters(file_path, joint_name=None):
    """
    Get parameters for a motor joint from a YAML file.

    Args:
        file_path (str): Path to the YAML file
        joint_name (str): Joint name

    Returns:
        dict: Parameters for all joints or a specific joint
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
    """
    Create a status table for the current joint operation.

    Args:
        joint_name (str): Joint name
        position (float): Current joint position
        status (str): Current status

    Returns:
        Panel: Rich Panel object
    """

    table = Table(box=box.ROUNDED, show_header=False, width=60)
    table.add_column("Property", style="cyan")
    table.add_column("Value", style="yellow")
    
    table.add_row("Joint", joint_name)
    table.add_row("Position", f"{position:.4f} rad" if position is not None else "Unknown")
    table.add_row("Status", status)
    
    return Panel(table, title="[bold blue]Joint Status", border_style="blue")

def calculate_crc(id, data):
    """
    Calculate CRC by summing ID and data.
    """
    return (id + sum(data)) & 0xFF

def check_crc(id, data):
    """
    Verify CRC of received message.
    """
    checksum = id + sum(data[:-1])
    return data[-1] == (checksum & 0xFF)

class TerminalControl:
    """Class to disable terminal echo and canonical mode."""
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def disable_echo(self):
        """Disable terminal echo and canonical mode."""
        tty.setcbreak(self.fd)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def restore_echo(self):
        """Restore terminal echo and canonical mode."""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

class EmergencyKeyWatcher:
    def __init__(self, robot_controller, emergency_key='space'):
        """
        Watch for a specific key press and trigger an emergency stop.
        
        Args:
            robot_controller (RobotController): RobotController object
            emergency_key (str): Key to trigger emergency stop

        Returns:
            None
        """
        self.robot_controller = robot_controller
        self.emergency_key = emergency_key
        self.running = True

    def start(self):
        """
        Set up the keyboard hook to watch for the emergency key.
        """
        console.print(f"[red][EmergencyKeyWatcher] Press [bold]'{self.emergency_key.upper()}'[/] to activate emergency stop.[/]")
        keyboard.on_press_key(self.emergency_key, self._callback)

    def _callback(self, event):
        """
        This callback is invoked immediately when the emergency key is pressed.

        Args:
            event (keyboard.KeyboardEvent): Keyboard event object

        Returns:
            None
        """
        console.print("[EmergencyKeyWatcher] Emergency key pressed!\n")
        self.robot_controller.emergency_stop()
        self.running = False
        keyboard.unhook_all()
        os._exit(1)

    def run(self):
        """
        Run the emergency key watcher. This loop runs in its own thread.
        """
        self.start()
        while self.running:
            time.sleep(0.1)

class RobotController:
    def __init__(self):
        self.emergency_stopped = False
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

        # Dictionary to track each joint’s status for our Live table
        self.joint_statuses = {}
        for joint_name in self.joint_info:
            self.joint_statuses[joint_name] = {
                "Homed": False,
                "Manual Adjustment": False,
                "Home Position": None,
                "Opposite Limit": None,
                "Zero Position": None
            }

    def _build_live_table(self):
        """ Build a Live Table for the current joint statuses.

        Returns:
            Table: Live Table object
        """
        table = Table(title="Joint Configuration Status", box=box.DOUBLE)
        table.add_column("Joint Name", justify="center")
        table.add_column("Homed", justify="center")
        table.add_column("Manual Adjust", justify="center")
        table.add_column("Home Position", justify="center")
        table.add_column("Opposite Limit", justify="center")
        table.add_column("Zero Position", justify="center")  # <--- new column
        
        for joint_name, status_dict in self.joint_statuses.items():
            homed_str = "[green]Yes[/]" if status_dict["Homed"] else "[red]No[/]"
            manual_str = "[yellow]In-progress[/]" if status_dict["Manual Adjustment"] else "[dim]---[/]"
            home_pos = status_dict["Home Position"] or "[dim]---[/]"
            opp_limit = status_dict["Opposite Limit"] or "[dim]---[/]"
            zero_pos = status_dict["Zero Position"] or "[dim]---[/]"   # <--- show zero position or ---
            
            table.add_row(
                joint_name,
                homed_str,
                manual_str,
                home_pos,
                opp_limit,
                zero_pos
            )
        return table

    def decode_int48(self, data):
        """ Decode 48-bit signed integer from 6 bytes.
        
        Args:
            data (bytes): 6 bytes of data
            
        Returns:
            float: Motor angle in degrees
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
    
    def receive_message(self, message):
        """ Receive and store the CAN message.
        
        Args:
            message (can.Message): Received CAN message
            
        Returns:
            None
        """
        self.can_resp = message

    def send_command(self, motor_id, command):
        """ Send a command to the motor and wait for response.
        
        Args:
            motor_id (int): Motor ID
            command (list): Command to send
            
        Returns:
            can.Message: Received CAN message
        """
        self.can_resp = False
        self.notifier.add_listener(self.receive_message)
        
        msg = can.Message(
            arbitration_id=motor_id,
            data=bytearray(command) + bytes([calculate_crc(motor_id, command)]),
            is_extended_id=False
        )
        self.bus.send(msg)
        
        ts = time.time()
        while not self.can_resp and time.time() - ts < TIMEOUT:
            time.sleep(0.01)
        
        self.notifier.remove_listener(self.receive_message)
        
        if (not self.can_resp 
            or not check_crc(motor_id, self.can_resp.data) 
            or motor_id != self.can_resp.arbitration_id):
            console.print("\n[bold red]ERROR[/] - No message received from Arctos arm")
            return None
        return self.can_resp
    
    def get_joint_name_from_id(self, motor_id):
        """ Get joint name from motor ID.
        
        Args:
            motor_id (int): Motor ID
            
        Returns:
            str: Joint name"""
        
        for joint_name, params in self.parameters.items():
            if params.get("motor_id") == motor_id:
                return joint_name
        return None

    def get_encoder_position(self, joint_name):
        """ Get the encoder position for a joint.
        
        Args:
            joint_name (str): Joint name
            
        Returns:
            float: Motor angle in radians
        """
        motor_id = self.parameters[joint_name].get("motor_id")
        response = self.send_command(motor_id, [0x31])  # read encoder

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

    def home_joint(self, motor_id):
        """ Home the motor joint.

        Args:
            motor_id (int): Motor ID

        Returns:
            None
        """
        console.print(f"\n[bold blue]Sending home command to motor {motor_id}...")
        
        homing_complete = Event()

        def response_listener(message):
            if message.arbitration_id == motor_id and len(message.data) > 1 and message.data[1] == 2:
                console.print(f"[bold green]✓ Motor {motor_id} homed successfully!")
                homing_complete.set()

        self.notifier.add_listener(response_listener)

        try:
            self.send_command(motor_id, [0x91])  # 0x91 in hex
            if not homing_complete.wait(timeout=300):
                console.print(f"\n[bold red]ERROR: Homing for motor {motor_id} timed out!")
        finally:
            self.notifier.remove_listener(response_listener)
        
    def emergency_stop(self):
        """ Activate emergency stop for all motors.

        Returns:
            None
        """
        if not self.emergency_stopped:
            self.emergency_stopped = True
            
            motor_ids = [self.parameters[joint_name]["motor_id"] for joint_name in self.joint_info]
            print("[RobotController] Emergency stop activated!")
            # Send the emergency stop command (example command [0xF7]) to all motors
            for motor_id in motor_ids:
                self.send_command(motor_id, [0xF7])
        else:
            print("[RobotController] Emergency stop already activated.")



    def manual_move_joint(self, motor_id, live):
        """ Manually adjust the joint position and set zero position.
        
        Args:
            motor_id (int): Motor ID
            live (Live): Live object for updating the table
            
        Returns:
            None
        """
        joint_name = self.get_joint_name_from_id(motor_id)
        degree_per_pulse = 1.8 / 16 / self.parameters[joint_name]["gear_ratio"]
        pulses = round(1 / degree_per_pulse)

        console.print(f"\n[bold blue]Please move the joint and align the arrows located on each part of the joint. {joint_name}.")
        console.print(f"The arrows should be pointing towards each other.")
        console.print(
            "[yellow]"
            "Use [bold]LEFT[/]/[bold]RIGHT[/] to move and [bold]ENTER[/] to save and set your zero position."
            "[/]"
        )

        # Mark "Manual Adjustment" in-progress
        self.joint_statuses[joint_name]["Manual Adjustment"] = True
        live.update(self._build_live_table())

        terminal_control = TerminalControl()
        terminal_control.disable_echo()

        def on_key_event(e):
            inverted = bool(self.parameters[joint_name].get("inverted"))

            if e.event_type == keyboard.KEY_DOWN:
                if e.name in ['left', 'right']:
                    direction = 0x80 if (e.name == 'left') ^ inverted else 0
                    self.relative_motion_by_pulses(motor_id, pulses, 100, 1, direction)
                    encoder_position = self.get_encoder_position(joint_name)
                    if encoder_position is not None:
                        # Update the "Zero Position" column in real time
                        self.joint_statuses[joint_name]["Zero Position"] = f"{encoder_position:.4f}"
                        # Redraw the table
                        live.update(self._build_live_table())

                elif e.name == 'enter':
                    encoder_position = self.get_encoder_position(joint_name)
                    console.print(
                        f"\n[blue]Zero position set at {encoder_position:.4f} radians for {joint_name}.[/]"
                    )
                    self.send_command(motor_id, [0x92])
                    update_yaml_parameters(
                        ARCTOS_YAML_PATH, joint_name, {"zero_position": encoder_position}
                    )
                    # Mark "Manual Adjustment" done
                    self.joint_statuses[joint_name]["Manual Adjustment"] = False
                    self.joint_statuses[joint_name]["Zero Position"] = f"{encoder_position:.4f}"
                    # Refresh table one last time
                    live.update(self._build_live_table())
                    self.stop_event.set()

        hook = keyboard.hook(on_key_event, suppress=False)
        try:
            self.stop_event.wait()
        finally:
            keyboard.unhook(hook)
            terminal_control.restore_echo()
            self.stop_event.clear()

    def move_to_zero_position(self):
        """ Move all joints to zero position and set zero position from the zero_position parameter found in the YAML file.

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
                    if len(message.data) > 1 and message.data[1] == 2:
                        console.print(f"[bold green]✓ Joint {motor_id} moved to zero position!")
                        movement_complete.set()
                    if len(message.data) > 1 and message.data[1] == 3:
                        raise Exception(
                            f"Endstop limit for motor ID {motor_id} triggered. Failed to move to zero position."
                        )

            if zero_position is not None:
                self.notifier.add_listener(response_listener)

                zero_position_enc = round(
                    zero_position * gear_ratio * (180 / math.pi) * ENCODER_RESOLUTION / 360
                )
                if inverted:
                    zero_position_enc = -zero_position_enc

                try:
                    self.move_to_absolute_position(motor_id, zero_position_enc)
                    console.status(f"[bold cyan]Waiting for joint {joint_name} to move to zero_position...")
                    if not movement_complete.wait(timeout=300):
                        console.print(f"\n[bold red]ERROR: Homing for joint {joint_name} timed out!")
                    if movement_complete.is_set():
                        time.sleep(1)
                        self.send_command(motor_id, [0x92])
                        console.print(f"[bold green]✓ Joint {motor_id} set to zero position!")
                finally:
                    self.notifier.remove_listener(response_listener)

    def move_to_absolute_position(self, motor_id, abs_position, speed=150, acceleration=2):
        """ Move the motor to an absolute position.
        
        Args:
            motor_id (int): Motor ID
            abs_position (int): Absolute position in pulses
            speed (int): Speed in pulses per second
            acceleration (int): Acceleration in pulses per second squared
            
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

    def relative_motion_by_pulses(self, motor_id, pulses, speed, acceleration, direction):
        """ Move the motor by a relative number of pulses.
        
        Args:
            motor_id (int): Motor ID
            pulses (int): Number of pulses to move
            speed (int): Speed in pulses per second
            acceleration (int): Acceleration in pulses per second squared
            direction (int): Direction of motion
            
        Returns:
            can.Message: Received CAN message
        """
        command = [
            0xFD,
            direction + ((speed >> 8) & 0b1111),
            speed & 0xFF,
            acceleration,
            (pulses >> 16) & 0xFF,
            (pulses >> 8) & 0xFF,
            pulses & 0xFF,
        ]
        return self.send_command(motor_id, command)

    def find_home_position(self, motor_id):
        """Find home position by moving until limit switch is hit.
        
        Args:
            motor_id (int): Motor ID
            
        Returns:
            float: Encoder position in radians
        """
        joint_name = self.get_joint_name_from_id(motor_id)
        degree_per_pulse = 1.8 / 16 / self.parameters[joint_name]["gear_ratio"]
        step_in_degree = 360
        pulses = round(step_in_degree / degree_per_pulse)
        inverted = self.parameters[joint_name].get("inverted")
        direction = 0x80 if inverted else 0

        console.print(f"\n[cyan]Finding home position for {joint_name}...")

        encoder_position = None
        while True:
            response = self.relative_motion_by_pulses(motor_id, pulses, 50, 1, direction)
            if response and len(response.data) > 1 and response.data[1] == 3:
                encoder_position = self.get_encoder_position(joint_name)
                if encoder_position is not None:
                    console.print(
                        f"[green]{joint_name} home position found at: {encoder_position:.4f} radians[/]"
                    )
                    # Record in joint_statuses
                    self.joint_statuses[joint_name]["Home Position"] = f"{encoder_position:.4f}"
                    break
            time.sleep(0.5)

        return encoder_position

    def find_opposite_limit(self, motor_id):
        """Find the opposite limit switch by moving in the opposite direction.
        
        Args:
            motor_id (int): Motor ID
            
        Returns:
            float: Encoder position in radians
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
        encoder_position = None
        while True:
            response = self.relative_motion_by_pulses(motor_id, pulses, 50, 1, direction)
            if response and len(response.data) > 1 and response.data[1] == 3:
                encoder_position = self.get_encoder_position(joint_name)
                if encoder_position is not None:
                    console.print(
                        f"[green]{joint_name} opposite limit found at: {encoder_position:.4f} radians[/]"
                    )
                    # Record in joint_statuses
                    self.joint_statuses[joint_name]["Opposite Limit"] = f"{encoder_position:.4f}"
                    break
            time.sleep(0.5)

        return encoder_position

    def configure_joints(self):
        """
        Home each joint, allow manual adjustment, and find the limits.
        Results saved to YAML.

        Returns:
            None
        """
        try:
            terminal_control = TerminalControl()
            terminal_control.disable_echo()
            
            console.print("\n[bold cyan]Starting joint configuration...[/]")

            # Start a Live display so the table updates at each step
            initial_table = self._build_live_table()
            with Live(initial_table, console=console, refresh_per_second=4) as live:

                for joint_name in self.joint_info:
                    console.rule(f"[bold blue]Configuring {joint_name}")
                    motor_id = self.parameters[joint_name].get("motor_id")

                    # Step 1: Home the joint
                    console.print(f"\n[green]Step 1: Homing {joint_name}...")
                    self.home_joint(motor_id)
                    # Mark that joint as 'Homed'
                    self.joint_statuses[joint_name]["Homed"] = True
                    live.update(self._build_live_table())

                    # Clear the stop event for the current joint
                    self.stop_event.clear()

                    # Step 2: Manual adjustment
                    console.print(f"\n[green]Step 2: Manual adjustment of {joint_name}...")
                    self.manual_move_joint(motor_id, live)
                    live.update(self._build_live_table())

                    # Step 3: Find limits
                    console.print(f"\n[green]Step 3: Finding limits for {joint_name}...")
                    home_encoder_position = self.find_home_position(motor_id)
                    live.update(self._build_live_table())
                    opposite_encoder_position = self.find_opposite_limit(motor_id)
                    live.update(self._build_live_table())
                    if motor_id != 3:
                        self.move_to_absolute_position(motor_id, 0)

                    # Update YAML
                    update_yaml_parameters(ARCTOS_YAML_PATH, joint_name, {
                        "home_position": home_encoder_position,
                        "opposite_limit": opposite_encoder_position
                    })

                    console.print(f"[bold green]✓ {joint_name} configuration complete![/]\n")

                    # Also re-draw the table after each joint is done
                    live.update(self._build_live_table())
                    console.clear()

                console.print("[bold green]✓ All joints configured successfully![/]")

        finally:
            terminal_control.restore_echo()

    def shutdown(self):
        """ Shutdown the robot controller and close the CAN bus.
        
        Returns:
            None
        """
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        console.print("[bold green]✓[/] Shutdown complete")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Arctos Robot Controller")
    parser.add_argument("--init", action="store_true", help="Configure joint limits and zero positions (Do this first)")
    parser.add_argument("--set-zero", action="store_true", help="Set zero position for all joints")
    parser.add_argument("--joint", type=str, nargs="+", help="Joint names to set zero position")
    parser.add_argument("--config", type=str, help="Path to the YAML configuration file")

    args = parser.parse_args()

    try:
        controller = RobotController()

        if args.config:
            ARCTOS_YAML_PATH = args.config

        if args.joint:
            controller.joint_info = [f"{joint}_joint" for joint in args.joint]

        emergency_watcher = EmergencyKeyWatcher(controller, emergency_key='esc')
        watcher_thread = Thread(target=emergency_watcher.run, daemon=True)
        watcher_thread.start()

        if args.set_zero:
            controller.move_to_zero_position()

        elif args.init:
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
