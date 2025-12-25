# Quick Build Reference

## Windows (Arduino CLI)

```cmd
cd ptz_controller_r4
build_and_upload.bat COM3
```

Replace `COM3` with your COM port (check Device Manager).

## Linux/macOS (Arduino CLI)

```bash
cd ptz_controller_r4
chmod +x build_and_upload.sh
./build_and_upload.sh /dev/ttyACM0
```

Replace `/dev/ttyACM0` with your port.

## Arduino IDE

1. Open `ptz_controller_r4.ino` in Arduino IDE
2. **Tools → Board → Arduino UNO R4 Boards → Arduino UNO R4 Minima**
3. **Tools → Port → [Select your port]**
4. Click **Upload** (arrow icon)
5. Open **Serial Monitor** at 115200 baud

## Find Your Port

**Windows**: Device Manager → Ports (COM & LPT)  
**Linux**: `ls /dev/tty* | grep -i usb`  
**macOS**: `ls /dev/cu.*`

## First Upload

After upload, open Serial Monitor (115200 baud). You should see:
```
PTZ Controller Ready
Commands: VEL, GOTO, SAVE, HOME, STOP, PRECISION, POS, STATUS
```

## Troubleshooting

- **Port not found**: Check USB cable, install drivers
- **Upload fails**: Close other serial programs, press RESET button
- **No serial output**: Check baud rate (115200), press RESET

See `BUILD_INSTRUCTIONS.md` for detailed help.


