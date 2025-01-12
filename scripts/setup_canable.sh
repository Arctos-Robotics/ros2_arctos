#!/bin/bash

# Define variables
INTERFACE="can0"
CANABLE_IDENTIFIER="CANable"  # Identifier for the CANable device
BITRATE="500000"  # Bitrate in bps (e.g., 500000 for 500 kbps)
QLEN="1000"

# Ensure the script is running as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root. Please use 'sudo ./setup_canable.sh'"
    exit 1
fi

# Detect the operating system
OS=$(uname)
echo "Detected operating system: $OS"

# Check for required tools based on the OS
if [[ "$OS" == "Linux" ]]; then
    if ! command -v slcand &> /dev/null || ! command -v ip &> /dev/null; then
        echo "slcand and ip are required. Install them with:"
        echo "  sudo apt-get install can-utils"
        exit 1
    fi
elif [[ "$OS" == "Darwin" ]]; then
    if ! command -v slcan_attach &> /dev/null; then
        echo "slcan_attach is required. Install it with:"
        echo "  brew install can-utils"
        exit 1
    fi
else
    echo "Unsupported operating system: $OS"
    exit 1
fi

# Find the CANable device
if [[ "$OS" == "Linux" ]]; then
    CANABLE_DEVICE=$(ls /dev/serial/by-id/ 2>/dev/null | grep -i "$CANABLE_IDENTIFIER")
    if [ -n "$CANABLE_DEVICE" ]; then
        TTY_DEVICE="/dev/serial/by-id/$CANABLE_DEVICE"
    else
        TTY_DEVICE=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)
    fi
elif [[ "$OS" == "Darwin" ]]; then
    TTY_DEVICE=$(ls /dev/tty.usbmodem* 2>/dev/null | grep -i "$CANABLE_IDENTIFIER" | head -n 1)
fi

if [ -z "$TTY_DEVICE" ]; then
    echo "No CANable device found. Ensure the device is connected."
    exit 1
fi

echo "Found CANable device: $TTY_DEVICE"

# Setup CANable device
if [[ "$OS" == "Linux" ]]; then
    echo "Attaching $TTY_DEVICE as $INTERFACE..."
    sudo slcand -o -c -s3 "$TTY_DEVICE" "$INTERFACE"
    echo "Bringing down $INTERFACE (if up)..."
    sudo ip link set down "$INTERFACE" 2>/dev/null || true
    echo "Setting bitrate for $INTERFACE to $BITRATE..."
    sudo ip link set "$INTERFACE" type can bitrate "$BITRATE"
    echo "Setting qlen for $INTERFACE to $QLEN"
    sudo ip link set "$INTERFACE" txqueuelen "$QLEN"
    echo "Bringing up $INTERFACE..."
    sudo ip link set up "$INTERFACE"
elif [[ "$OS" == "Darwin" ]]; then
    echo "Attaching $TTY_DEVICE as $INTERFACE with $BITRATE bitrate..."
    sudo slcan_attach -o -c -s3 "$TTY_DEVICE" "$INTERFACE"
    echo "Bringing up $INTERFACE..."
    sudo slcan_up "$INTERFACE"
fi

# Verify the setup
if [[ "$OS" == "Linux" ]]; then
    echo "Verifying $INTERFACE status..."
    ip -details link show "$INTERFACE"
elif [[ "$OS" == "Darwin" ]]; then
    echo "Verifying $INTERFACE status..."
    ifconfig "$INTERFACE"
fi

echo "Setup complete! $INTERFACE is up and running at $BITRATE."
