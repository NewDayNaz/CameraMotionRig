# Quick Start Guide - FYSETC E4 Firmware

## Prerequisites Checklist

- [ ] ESP-IDF v5.0+ installed
- [ ] FYSETC E4 board connected via USB
- [ ] COM port identified (Windows) or device path (Linux/macOS)

## Windows Quick Start

1. **Open ESP-IDF Command Prompt**
   - Use the shortcut created by ESP-IDF installer
   - Or run: `%userprofile%\esp\esp-idf\export.bat`

2. **Navigate to project**
   ```cmd
   cd path\to\camera_fysetc_e4_
   ```

3. **Build and flash (automated)**
   ```cmd
   build_and_flash.bat COM3
   ```
   Replace `COM3` with your actual COM port (check Device Manager)

4. **Or manually:**
   ```cmd
   idf.py set-target esp32
   idf.py build
   idf.py -p COM3 flash monitor
   ```

## Linux/macOS Quick Start

1. **Set ESP-IDF environment**
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

2. **Navigate to project**
   ```bash
   cd path/to/camera_fysetc_e4_
   ```

3. **Make script executable and run**
   ```bash
   chmod +x build_and_flash.sh
   ./build_and_flash.sh /dev/ttyUSB0
   ```
   Replace `/dev/ttyUSB0` with your actual device (try `/dev/ttyACM0` if needed)

4. **Or manually:**
   ```bash
   idf.py set-target esp32
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

## First Time Flashing

For first-time flashing, you may want to erase the flash first:

```bash
idf.py -p COM3 erase-flash    # Windows
idf.py -p /dev/ttyUSB0 erase-flash  # Linux/macOS
```

## Finding Your Serial Port

**Windows:**
- Device Manager → Ports (COM & LPT)
- Look for "USB Serial" or "CP210x" or "CH340"
- Port will be listed as COM3, COM7, etc.

**Linux:**
```bash
ls /dev/ttyUSB* /dev/ttyACM*
# Or:
dmesg | grep tty
```

**macOS:**
```bash
ls /dev/cu.*
# Or:
ls /dev/tty.*
```

## Testing After Flash

1. Open serial monitor: `idf.py -p COM3 monitor`
2. You should see startup logs
3. Send test command: `POS` (should return position)
4. Send: `STATUS` (should return system status)

## Common Issues

**"idf.py: command not found"**
- ESP-IDF environment not set - run export script

**"Permission denied" (Linux/macOS)**
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Or use sudo: `sudo idf.py -p /dev/ttyUSB0 flash`

**"Could not open port"**
- Check port number/name
- Unplug and replug USB cable
- Try different USB port
- Check USB cable (must support data, not just power)

**Flash fails / Board not responding**
- Put board in download mode:
  1. Hold BOOT button
  2. Press and release RESET button
  3. Release BOOT button
- Try lower baud rate: `idf.py -p COM3 flash -b 115200`

## Next Steps

- See [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) for detailed documentation
- See [README.md](README.md) for firmware features and usage
- Test with commands: `POS`, `STATUS`, `VEL 100 0 0`, `HOME`

