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

echo "Step 1: Verifying partition table file..."
if [ ! -f "partitions.csv" ]; then
    echo "ERROR: partitions.csv not found in project root!"
    exit 1
fi
echo "Partition table file found: partitions.csv"
echo ""
echo "Step 1b: Checking flash size (optional)..."
echo "To check your board's actual flash size, run: idf.py -p $PORT flash_id"
echo "Or check bootloader output when device starts."
echo "Current configuration: 16MB flash (adjust in sdkconfig.defaults if different)"
echo ""

# Check if we're doing OTA - if so, skip fullclean/reconfigure (partition table won't change)
if [ "$FLASH_MODE" = "OTA" ]; then
    echo "Step 2: Building firmware for OTA update..."
    echo "NOTE: Partition table is only used for build - OTA cannot change partitions."
    echo ""
    idf.py build
else
    echo "Step 2: Cleaning and reconfiguring to ensure partition table is updated..."
    echo "Deleting sdkconfig to force regeneration from sdkconfig.defaults..."
    rm -f sdkconfig sdkconfig.old
    idf.py fullclean
    idf.py reconfigure
    
    echo ""
    echo "Step 3: Building firmware with new partition table..."
    idf.py build
fi

if [ "$FLASH_MODE" = "OTA" ]; then
    echo ""
    echo "Step 3: Uploading firmware via OTA..."
    echo ""
    echo "NOTE: OTA updates only change the firmware binary, NOT the partition table."
    echo "      The partition table on the device was set during initial serial flash."
    echo "      If you need to change partitions, you must flash via serial."
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
    echo "Step 4: Erasing flash to ensure clean partition table..."
    echo "WARNING: This will erase all data on the device!"
    echo ""
    read -p "Erase flash? (y/N): " ERASE_CONFIRM
    if [[ "$ERASE_CONFIRM" =~ ^[Yy]$ ]]; then
        echo "Erasing flash..."
        idf.py -p "$PORT" erase-flash
        if [ $? -ne 0 ]; then
            echo "Flash erase failed!"
            exit 1
        fi
        echo "Flash erased successfully."
        echo ""
    else
        echo "Skipping flash erase. If OTA doesn't work, try erasing flash first."
        echo ""
    fi
    
    echo ""
    echo "Step 5: Flashing partition table and firmware to $PORT..."
    echo ""
    echo "Note: With OTA partition table, firmware will be flashed to ota_0 partition"
    echo "      and automatically set as the boot partition."
    echo ""
    echo "If flashing fails, you may need to manually enter bootloader mode:"
    echo "  1. Connect GPIO0 pin to GND using a jumper wire"
    echo "  2. Press and release the RESET button"
    echo "  3. Remove the GPIO0 to GND connection"
    echo "  4. Wait 1 second, then run: idf.py -p $PORT flash"
    echo ""
    
    # Verify partition table binary exists
    if [ ! -f "build/partition_table/partition-table.bin" ]; then
        echo "ERROR: Partition table binary not found!"
        echo "Make sure the build completed successfully."
        exit 1
    fi
    
    echo "Partition table binary found: build/partition_table/partition-table.bin"
    echo ""
    
    # Flash partition table and firmware together
    # The -p flag ensures partition table is flashed
    idf.py -p "$PORT" flash
    
    if [ $? -ne 0 ]; then
        echo ""
        echo "Flash failed!"
        echo "Make sure:"
        echo "  - Board is connected to $PORT"
        echo "  - You have permission to access the port (may need sudo or add user to dialout group)"
        echo "  - USB cable supports data transfer"
        echo "  - Hold BOOT button, press RESET, release RESET, release BOOT if needed"
        exit 1
    fi
    
    echo ""
    echo "Step 6: Verifying partition table..."
    echo "Running partition table info command..."
    idf.py -p "$PORT" partition-table
    
    echo ""
    echo "Step 7: Starting serial monitor..."
    echo "Press Ctrl+] to exit monitor"
    echo ""
    idf.py -p "$PORT" monitor
fi

