# Build and Upload Instructions

This document describes how to build and upload the PTZ Controller firmware to an Arduino Uno R4 Minima.

## Prerequisites

- **Arduino Uno R4 Minima** board
- **USB cable** (data-capable USB cable)
- **Windows, Linux, or macOS** computer

## Method 1: Using Arduino CLI (Recommended)

### Windows

1. **Install Arduino CLI** (if not already installed):
   ```powershell
   winget install Arduino.ArduinoCLI
   ```
   Or download from: https://arduino.github.io/arduino-cli/

2. **Run the build script**:
   ```cmd
   cd ptz_controller_r4
   build_and_upload.bat COM3
   ```
   Replace `COM3` with your actual COM port (check Device Manager).

### Linux/macOS

1. **Install Arduino CLI**:
   ```bash
   curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
   ```

2. **Make script executable**:
   ```bash
   chmod +x build_and_upload.sh
   ```

3. **Run the build script**:
   ```bash
   cd ptz_controller_r4
   ./build_and_upload.sh /dev/ttyACM0
   ```
   Replace `/dev/ttyACM0` with your actual port (check with `ls /dev/tty*`).

   **Note**: On Linux, you may need to add your user to the `dialout` group:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in for changes to take effect
   ```

## Method 2: Using Arduino IDE

### Setup

1. **Install Arduino IDE 2.x**:
   - Download from: https://www.arduino.cc/en/software
   - Install and launch Arduino IDE

2. **Install Arduino Uno R4 Board Support**:
   - Open Arduino IDE
   - Go to **Tools → Board → Boards Manager**
   - Search for "Arduino UNO R4"
   - Install "Arduino UNO R4 Boards" by Arduino

3. **Open the Project**:
   - In Arduino IDE, go to **File → Open**
   - Navigate to `ptz_controller_r4/`
   - Open `ptz_controller_r4.ino`

### Build and Upload

1. **Select Board**:
   - Go to **Tools → Board → Arduino UNO R4 Boards → Arduino UNO R4 Minima**

2. **Select Port**:
   - Go to **Tools → Port**
   - Select the COM port (Windows) or `/dev/ttyACM0` (Linux) or `/dev/cu.usbmodem*` (macOS)
   - If port doesn't appear, check USB cable and drivers

3. **Verify/Compile**:
   - Click the **Verify** button (checkmark icon) or press `Ctrl+R`
   - Wait for compilation to complete
   - Check for errors in the output panel

4. **Upload**:
   - Click the **Upload** button (arrow icon) or press `Ctrl+U`
   - Wait for upload to complete
   - You should see "Upload successful" message

5. **Open Serial Monitor**:
   - Click the **Serial Monitor** button (magnifying glass icon) or press `Ctrl+Shift+M`
   - Set baud rate to **115200**
   - You should see: `PTZ Controller Ready`

## Known Issue: Internal Compiler Error

If you encounter an "internal compiler error" in `Binary.h`, this is a **known bug** in the Arduino Uno R4 board support package (Renesas core v1.5.1). 

**Quick Fix:**
1. Update the core: `arduino-cli core upgrade arduino:renesas_uno`
2. Or use Arduino IDE 2.3.0+ which may have a fixed compiler
3. See `TROUBLESHOOTING.md` for detailed solutions

## Troubleshooting

### Port Not Found

**Windows**:
- Check Device Manager → Ports (COM & LPT)
- Look for "Arduino UNO R4" or "USB Serial Device"
- Install drivers if needed (usually automatic)

**Linux**:
- Check available ports: `ls /dev/tty*`
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- May need to unplug and replug USB cable

**macOS**:
- Check available ports: `ls /dev/cu.*`
- Port usually appears as `/dev/cu.usbmodem*`

### Upload Fails

1. **Check USB Cable**:
   - Use a data-capable USB cable (not charge-only)
   - Try a different USB cable

2. **Check Port Selection**:
   - Ensure correct COM port is selected
   - Close other programs using the serial port (Serial Monitor, PuTTY, etc.)

3. **Reset Board**:
   - Press and release the RESET button on the board
   - Try uploading again immediately after reset

4. **Bootloader Mode**:
   - Some boards may need to be put in bootloader mode
   - For R4 Minima: Press and hold RESET, then press and hold BOOT (if available), release RESET, then release BOOT

### Compilation Errors

1. **Missing Board Support**:
   - Ensure "Arduino UNO R4 Boards" is installed
   - In Arduino IDE: Tools → Board → Boards Manager

2. **Missing Files**:
   - Ensure all `.cpp` and `.h` files are in the same directory as `ptz_controller_r4.ino`
   - Arduino IDE requires all files to be in the sketch folder

3. **Library Issues**:
   - This project uses only standard Arduino libraries
   - No external libraries required

### Serial Monitor Shows Nothing

1. **Check Baud Rate**:
   - Must be set to **115200**

2. **Check Connection**:
   - Ensure USB cable is connected
   - Try unplugging and replugging

3. **Check Board**:
   - Press RESET button on board
   - Should see "PTZ Controller Ready" message

## Quick Reference

### Arduino CLI Commands

```bash
# List connected boards
arduino-cli board list

# Compile
arduino-cli compile --fqbn arduino:renesas_uno:unor4_minima ptz_controller_r4.ino

# Upload
arduino-cli upload -p COM3 --fqbn arduino:renesas_uno:unor4_minima ptz_controller_r4.ino

# Monitor
arduino-cli monitor -p COM3 -c baudrate=115200
```

### Finding Your Port

**Windows**:
- Device Manager → Ports (COM & LPT)
- Or: `mode` command in Command Prompt

**Linux**:
```bash
ls /dev/tty* | grep -i usb
# or
dmesg | grep tty
```

**macOS**:
```bash
ls /dev/cu.*
```

## Next Steps

After successful upload:
1. Open Serial Monitor at 115200 baud
2. You should see: `PTZ Controller Ready`
3. Try commands: `HOME PAN`, `HOME TILT`, `VEL 0.5 0.0 0.0`
4. See `QUICK_START.md` for more commands

