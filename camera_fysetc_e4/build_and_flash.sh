#!/bin/bash
# Build and flash script for FYSETC E4 firmware (Linux/macOS)
# Usage: ./build_and_flash.sh [PORT|OTA:IP_ADDRESS]
# Example: ./build_and_flash.sh /dev/ttyUSB0
# Example: ./build_and_flash.sh OTA:192.168.1.100

set -e

# Get port or OTA address from command line argument
TARGET=${1:-}

if [ -z "$TARGET" ]; then
    echo "Please specify target:"
    echo "  Serial flash: ./build_and_flash.sh /dev/ttyUSB0"
    echo "  OTA upload:   ./build_and_flash.sh OTA:192.168.1.100"
    exit 1
fi

# Check if OTA mode
if [[ "$TARGET" == OTA:* ]]; then
    FLASH_MODE="OTA"
    OTA_IP="${TARGET#OTA:}"
    echo "========================================"
    echo "FYSETC E4 Firmware Build and OTA Upload"
    echo "========================================"
    echo ""
    echo "Mode: OTA Update"
    echo "Target IP: $OTA_IP"
    echo ""
else
    FLASH_MODE="SERIAL"
    PORT="$TARGET"
    echo "========================================"
    echo "FYSETC E4 Firmware Build and Flash"
    echo "========================================"
    echo ""
    echo "Mode: Serial Flash"
    echo "Using port: $PORT"
    echo ""
fi

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

if [ "$FLASH_MODE" = "OTA" ]; then
    echo ""
    echo "Step 2: Uploading firmware via OTA..."
    echo ""
    
    # Find the firmware binary
    FIRMWARE_BIN="build/camera_fysetc_e4.bin"
    if [ ! -f "$FIRMWARE_BIN" ]; then
        echo "ERROR: Firmware binary not found at $FIRMWARE_BIN"
        echo "Make sure the build completed successfully."
        exit 1
    fi
    
    echo "Uploading $FIRMWARE_BIN to http://$OTA_IP/api/update"
    echo ""
    
    # Check if curl is available
    if ! command -v curl &> /dev/null; then
        echo "ERROR: curl is required for OTA upload"
        echo "Please install curl: sudo apt-get install curl  # Debian/Ubuntu"
        echo "                    brew install curl           # macOS"
        exit 1
    fi
    
    # Upload firmware using curl
    if [[ "$OSTYPE" == "darwin"* ]]; then
        FILE_SIZE=$(stat -f%z "$FIRMWARE_BIN" 2>/dev/null || echo "unknown")
    else
        FILE_SIZE=$(stat -c%s "$FIRMWARE_BIN" 2>/dev/null || echo "unknown")
    fi
    echo "File size: $FILE_SIZE bytes"
    echo "Uploading..."
    
    RESPONSE=$(curl -s -w "\n%{http_code}" -X POST \
        -H "Content-Type: application/octet-stream" \
        --data-binary "@$FIRMWARE_BIN" \
        "http://$OTA_IP/api/update")
    
    HTTP_CODE=$(echo "$RESPONSE" | tail -n1)
    BODY=$(echo "$RESPONSE" | sed '$d')
    
    echo ""
    echo "Response: $BODY"
    echo ""
    
    if [ "$HTTP_CODE" = "200" ] && echo "$BODY" | grep -q '"status":"ok"'; then
        echo "OTA update successful! Device will reboot automatically."
        echo "Wait 30 seconds and check the device."
    else
        echo "OTA update may have failed (HTTP $HTTP_CODE)"
        echo "Make sure:"
        echo "  - Device is connected to WiFi"
        echo "  - Device IP address is correct: $OTA_IP"
        echo "  - Device is accessible on the network"
        echo "  - HTTP server is running on the device"
        exit 1
    fi
else
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
fi

