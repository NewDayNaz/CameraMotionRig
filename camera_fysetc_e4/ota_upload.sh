#!/bin/bash
# OTA upload script for FYSETC E4 firmware (Linux/macOS)
# Usage: ./ota_upload.sh [IP_ADDRESS] [FIRMWARE_BIN]
# Example: ./ota_upload.sh 192.168.1.100
# Example: ./ota_upload.sh 192.168.1.100 build/camera_fysetc_e4.bin

set -e

# Get IP address and optional firmware path
if [ -z "$1" ]; then
    echo "Usage: ./ota_upload.sh [IP_ADDRESS] [FIRMWARE_BIN]"
    echo "Example: ./ota_upload.sh 192.168.1.100"
    echo ""
    read -p "Enter device IP address: " OTA_IP
else
    OTA_IP="$1"
fi

if [ -z "$2" ]; then
    FIRMWARE_BIN="build/camera_fysetc_e4.bin"
else
    FIRMWARE_BIN="$2"
fi

if [ ! -f "$FIRMWARE_BIN" ]; then
    echo "ERROR: Firmware binary not found at $FIRMWARE_BIN"
    echo "Make sure the build completed successfully or specify the correct path."
    exit 1
fi

echo "========================================"
echo "OTA Firmware Upload"
echo "========================================"
echo ""
echo "Target IP: $OTA_IP"
echo "Firmware: $FIRMWARE_BIN"
echo ""

# Check if curl is available
if ! command -v curl &> /dev/null; then
    echo "ERROR: curl is required for OTA upload"
    echo "Please install curl: sudo apt-get install curl  # Debian/Ubuntu"
    echo "                    brew install curl           # macOS"
    exit 1
fi

# Get file size
if [[ "$OSTYPE" == "darwin"* ]]; then
    FILE_SIZE=$(stat -f%z "$FIRMWARE_BIN" 2>/dev/null || echo "unknown")
else
    FILE_SIZE=$(stat -c%s "$FIRMWARE_BIN" 2>/dev/null || echo "unknown")
fi

echo "File size: $FILE_SIZE bytes"
echo "Uploading..."

# Upload firmware using curl
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

