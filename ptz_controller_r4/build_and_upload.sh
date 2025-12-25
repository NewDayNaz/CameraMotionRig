#!/bin/bash
# Build and upload script for PTZ Controller (Arduino Uno R4 Minima)
# Usage: ./build_and_upload.sh [PORT]
# Example: ./build_and_upload.sh /dev/ttyACM0

set -e

# Get port from command line argument, or prompt
if [ -z "$1" ]; then
    echo "Please specify port: ./build_and_upload.sh /dev/ttyACM0"
    echo "Or edit this script and set PORT below"
    PORT="/dev/ttyACM0"
else
    PORT="$1"
fi

echo "========================================"
echo "PTZ Controller Build and Upload"
echo "Arduino Uno R4 Minima"
echo "========================================"
echo ""
echo "Using port: $PORT"
echo ""

# Check if Arduino CLI is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "ERROR: Arduino CLI not found"
    echo ""
    echo "Install Arduino CLI:"
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo ""
    echo "Or use Arduino IDE:"
    echo "  Open ptz_controller_r4.ino in Arduino IDE"
    echo "  Select board: Arduino Uno R4 Minima"
    echo "  Select port: $PORT"
    echo "  Click Upload"
    echo ""
    exit 1
fi

echo "Step 1: Checking Arduino CLI configuration..."
arduino-cli config init 2>/dev/null || true
arduino-cli core update-index

echo ""
echo "Step 2: Installing Arduino Uno R4 Minima core (if needed)..."
arduino-cli core install arduino:renesas_uno

echo ""
echo "Step 3: Compiling firmware..."
if arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi ptz_controller_r4.ino; then
    FQBN="arduino:renesas_uno:unor4wifi"
elif arduino-cli compile --fqbn arduino:renesas_uno:unor4_minima ptz_controller_r4.ino; then
    FQBN="arduino:renesas_uno:unor4_minima"
else
    echo "Compilation failed!"
    echo ""
    echo "Make sure:"
    echo "  - All source files are in the same directory as ptz_controller_r4.ino"
    echo "  - Arduino Uno R4 core is installed"
    exit 1
fi

echo ""
echo "Step 4: Uploading firmware to $PORT..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" ptz_controller_r4.ino
if [ $? -ne 0 ]; then
    echo "Upload failed!"
    echo ""
    echo "Make sure:"
    echo "  - Board is connected to $PORT"
    echo "  - USB cable supports data transfer"
    echo "  - No other program is using the serial port"
    echo "  - You have permission to access $PORT (may need sudo or add user to dialout group)"
    echo "  - Board is in upload mode (may need to press RESET button)"
    echo ""
    exit 1
fi

echo ""
echo "========================================"
echo "Upload successful!"
echo "========================================"
echo ""
echo "Step 5: Opening serial monitor..."
echo "Press Ctrl+C to exit monitor"
echo ""
sleep 2
arduino-cli monitor -p "$PORT" -c baudrate=115200


