# Building and Uploading Firmware to FYSETC E4

This guide will help you build and upload the ESP-IDF firmware to your FYSETC E4 board.

## Prerequisites

### 1. Install ESP-IDF

You need ESP-IDF v5.0 or later installed. Follow the official ESP-IDF installation guide:

**Windows:**
1. Download ESP-IDF installer from: https://dl.espressif.com/dl/esp-idf/
2. Run the installer and follow the prompts
3. Select ESP-IDF v5.0 or later
4. The installer will set up the environment automatically

**Linux/macOS:**
```bash
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.0  # or later version
./install.sh esp32
. ./export.sh
```

### 2. Verify ESP-IDF Installation

Open a new terminal and verify ESP-IDF is installed:

```bash
idf.py --version
```

You should see output showing the ESP-IDF version.

## Building the Firmware

### Step 1: Navigate to Project Directory

```bash
cd camera_fysetc_e4_
```

### Step 2: Set ESP-IDF Environment (if not already set)

**Windows:**
- The ESP-IDF installer creates a "ESP-IDF Command Prompt" shortcut
- Use that, or run from a regular command prompt:
```cmd
%userprofile%\esp\esp-idf\export.bat
```

**Linux/macOS:**
```bash
. $HOME/esp/esp-idf/export.sh
```

### Step 3: Configure the Project

First-time setup - configure the project:

```bash
idf.py set-target esp32
idf.py menuconfig
```

In menuconfig, you can adjust:
- Serial flasher config → Default serial port (set to your COM port on Windows, e.g., COM3)
- Component config → Log output → Default log verbosity (set to Info or Debug for more output)

Press 'S' to save, 'Q' to quit.

**Note:** The `sdkconfig.defaults` file already contains good defaults, so you may not need to change much.

### Step 4: Build the Firmware

```bash
idf.py build
```

This will compile all source files and create the firmware binary. You should see:
```
Project build complete.
```

The compiled firmware will be in `build/camera_fysetc_e4.bin`

## Uploading to FYSETC E4 Board

### Step 1: Connect the Board

1. Connect the FYSETC E4 board to your computer via USB
2. On Windows, note the COM port number (e.g., COM3, COM7)
   - Check Device Manager → Ports (COM & LPT)
3. On Linux/macOS, note the device path (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`)
   - Check with: `ls /dev/tty*`

### Step 2: Set Serial Port

**Method 1: Command line (recommended)**
```bash
idf.py -p COM3 flash monitor  # Windows: use COM3, COM7, etc.
idf.py -p /dev/ttyUSB0 flash monitor  # Linux
idf.py -p /dev/ttyACM0 flash monitor  # macOS/Linux
```

**Method 2: Set in menuconfig**
```bash
idf.py menuconfig
```
Navigate to: Serial flasher config → Default serial port
Enter your port and save.

### Step 3: Flash the Firmware

**Erase flash first (recommended for first-time flashing):**
```bash
idf.py -p COM3 erase-flash  # Replace COM3 with your port
```

**Flash the firmware:**
```bash
idf.py -p COM3 flash  # Replace COM3 with your port
```

You should see output like:
```
...
Writing at 0x00010000... (100 %)
Wrote 123456 bytes (76879 compressed) at 0x00010000 in 6.8 seconds (effective 145.1 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
```

## Monitoring Serial Output

To see debug output and logs:

```bash
idf.py -p COM3 monitor  # Replace COM3 with your port
```

Press `Ctrl+]` to exit the monitor.

**Or combine flash + monitor:**
```bash
idf.py -p COM3 flash monitor
```

## Troubleshooting

### Port Not Found
- Check USB cable connection
- Install USB-to-Serial drivers if needed (CP210x or CH340 drivers)
- On Windows: Check Device Manager for COM port
- On Linux: Check permissions - may need to add user to dialout group:
  ```bash
  sudo usermod -a -G dialout $USER
  # Then logout and login again
  ```

### Build Errors
- Make sure ESP-IDF v5.0+ is installed
- Check that `IDF_PATH` environment variable is set
- Try: `idf.py fullclean` then rebuild
- Verify all source files are in `main/` directory

### Flash Errors
- Press and hold BOOT button on ESP32, then press RESET, release RESET, then release BOOT to enter download mode
- Check USB cable quality (data cable, not just power)
- Try different USB port
- Lower baud rate: `idf.py -p COM3 flash -b 115200`

### Permission Denied (Linux/macOS)
```bash
sudo chmod 666 /dev/ttyUSB0  # Replace with your port
# Or add user to dialout group (see above)
```

## Quick Reference Commands

```bash
# Build
idf.py build

# Build and flash
idf.py -p COM3 flash

# Flash and monitor
idf.py -p COM3 flash monitor

# Erase flash
idf.py -p COM3 erase-flash

# Clean build
idf.py fullclean

# Menuconfig
idf.py menuconfig
```

## Using USB Serial (Communication from Raspberry Pi)

After flashing, the ESP32 will communicate over USB serial at 115200 baud.

To test communication, use a serial terminal:
- **Windows**: PuTTY, Tera Term, or Arduino Serial Monitor
- **Linux/macOS**: `screen /dev/ttyUSB0 115200` or `minicom`

Send commands like:
```
POS
STATUS
VEL 100 0 0
HOME
```

## Additional Resources

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
- [FYSETC E4 Documentation](https://fysetc.github.io/E4/)
- [ESP-IDF Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)

