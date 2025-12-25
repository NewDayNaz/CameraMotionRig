#!/bin/bash
# Build and flash script for FYSETC E4 firmware (Linux/macOS)
# Usage: ./build_and_flash.sh [PORT]
# Example: ./build_and_flash.sh /dev/ttyUSB0

set -e

# Get port from command line argument or use default
PORT=${1:-/dev/ttyUSB0}

echo "========================================"
echo "FYSETC E4 Firmware Build and Flash"
echo "========================================"
echo ""
echo "Using port: $PORT"
echo ""

# Check if ESP-IDF environment is set
if ! command -v idf.py &> /dev/null; then
    echo "ERROR: ESP-IDF not found in PATH"
    echo "Please source ESP-IDF environment first:"
    echo "  . \$HOME/esp/esp-idf/export.sh"
    echo ""
    
    # Try to source if exists
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        echo "Sourcing ESP-IDF environment..."
        . "$HOME/esp/esp-idf/export.sh"
    else
        echo "ESP-IDF not found. Please install and set up ESP-IDF first."
        exit 1
    fi
fi

echo "Step 1: Building firmware..."
idf.py build

echo ""
echo "Step 2: Erasing flash (skipped by default)..."
# Uncomment the next line to erase flash (recommended for first-time flashing)
# idf.py -p "$PORT" erase-flash

echo ""
echo "Step 3: Flashing firmware to $PORT..."
idf.py -p "$PORT" flash

if [ $? -ne 0 ]; then
    echo "Flash failed!"
    echo "Make sure:"
    echo "  - Board is connected to $PORT"
    echo "  - You have permission to access the port (may need sudo or add user to dialout group)"
    echo "  - USB cable supports data transfer"
    echo "  - Hold BOOT button, press RESET, release RESET, release BOOT if needed"
    exit 1
fi

echo ""
echo "Step 4: Starting serial monitor..."
echo "Press Ctrl+] to exit monitor"
echo ""
idf.py -p "$PORT" monitor

