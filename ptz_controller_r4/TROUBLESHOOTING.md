# Troubleshooting Guide

## Internal Compiler Error (ICE) in Binary.h

If you see an error like:
```
internal compiler error: in decl_attributes, at attribs.c:453
   B0101001  DEPRECATED(0b0101001 ) = 41,
```

This is a **known bug** in the Arduino Uno R4 board support package (Renesas core), not an error in this firmware.

### Solutions (try in order):

#### 1. Update Board Support Package

**Arduino CLI:**
```cmd
arduino-cli core update-index
arduino-cli core upgrade arduino:renesas_uno
```

**Arduino IDE:**
- Tools → Board → Boards Manager
- Search for "Arduino UNO R4"
- Click "Update" if available
- Install latest version (1.5.2 or later)

#### 2. Use Arduino IDE Instead

Arduino IDE 2.3.0+ may use a different compiler version that avoids this bug:
1. Open `ptz_controller_r4.ino` in Arduino IDE
2. Tools → Board → Arduino UNO R4 Boards → Arduino UNO R4 Minima
3. Tools → Port → [Select your port]
4. Click Upload

#### 3. Clean and Rebuild

**Arduino CLI:**
```cmd
arduino-cli cache clean
arduino-cli core uninstall arduino:renesas_uno
arduino-cli core install arduino:renesas_uno
```

**Arduino IDE:**
- File → Preferences → Show verbose output during compilation
- Sketch → Export Compiled Binary
- Then try uploading again

#### 4. Check Compiler Version

The bug may be related to the GCC version. Arduino IDE 2.3.0+ uses a newer toolchain.

#### 5. Workaround: Use Different Board Variant

Try compiling for a different R4 variant:
```cmd
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi ptz_controller_r4.ino
```

Then manually change the FQBN in the upload command if needed.

#### 6. Report the Issue

If none of the above work:
- Report to Arduino: https://github.com/arduino/ArduinoCore-renesas/issues
- Check for existing issues and workarounds
- Consider using Arduino IDE 2.3.0+ which may have fixes

## Other Common Issues

### Port Not Found

**Windows:**
- Check Device Manager → Ports (COM & LPT)
- Install drivers if needed
- Try different USB cable/port

**Linux:**
- Add user to dialout: `sudo usermod -a -G dialout $USER`
- Log out and back in
- Check permissions: `ls -l /dev/ttyACM*`

**macOS:**
- Check System Preferences → Security & Privacy
- Allow access to USB devices

### Upload Fails

1. **Close Serial Monitor** - Only one program can use the port
2. **Press RESET button** on board before uploading
3. **Check USB cable** - Use data-capable cable (not charge-only)
4. **Try different USB port** - Some ports may not work

### Compilation Errors

1. **Missing files** - Ensure all `.cpp` and `.h` files are in the same directory as `.ino`
2. **Wrong board selected** - Must be "Arduino UNO R4 Minima"
3. **Outdated core** - Update board support package
4. **Arduino CLI version** - Update: `arduino-cli version` (should be 0.35.0+)

### Serial Monitor Shows Nothing

1. **Check baud rate** - Must be **115200**
2. **Press RESET** button on board
3. **Check connection** - Unplug and replug USB
4. **Try different terminal** - PuTTY, Tera Term, or Arduino Serial Monitor

## Getting Help

1. Check Arduino Forum: https://forum.arduino.cc/
2. Arduino R4 Issues: https://github.com/arduino/ArduinoCore-renesas/issues
3. Arduino CLI Issues: https://github.com/arduino/arduino-cli/issues

## Known Limitations

- **Internal Compiler Error**: Known bug in Renesas core v1.5.1, may be fixed in v1.5.2+
- **EEPROM size**: Arduino Uno R4 has limited EEPROM (1024 bytes), presets are stored efficiently
- **Step rate**: Maximum step rate is limited by `MIN_STEP_INTERVAL_US` (50us = 20kHz per axis)


