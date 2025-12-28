# FYSETC E4 Cinematic PTZ Camera Rig Firmware

ESP-IDF based firmware for controlling a 3-axis cinematic PTZ (Pan-Tilt-Zoom) camera rig using the FYSETC E4 board (ESP32 + TMC2209 stepper drivers).

## Hardware

- **Controller**: FYSETC E4 board
- **MCU**: ESP32
- **Drivers**: 4x onboard TMC2209 stepper drivers (Step/Dir control)
- **Axes**: 
  - PAN (X-axis socket: GPIO27 step, GPIO26 dir)
  - TILT (Y-axis socket: GPIO33 step, GPIO32 dir)
  - ZOOM (E0-axis socket: GPIO16 step, GPIO17 dir)
- **Endstops**: PAN min (GPIO34), TILT min (GPIO35), ZOOM min (GPIO15)
- **Sensorless (optional)**: ZOOM can use TMC2209 stallGuard if configured (see `homing.h` ZOOM_USE_SENSORLESS_HOMING)

## Features

- **Deterministic step generation** using GPTimer ISR at 40kHz
- **Quintic polynomial interpolation** for smooth, film-quality motion
- **Segment-based execution** (8ms segments) with DDA step distribution
- **Endstop homing** for PAN and TILT axes
- **Sensorless homing** for ZOOM axis (TMC2209 stallGuard)
- **Preset storage** using ESP32 NVS (up to 16 presets)
- **Manual velocity control** via USB serial from Raspberry Pi
- **Soft limits** with gradual speed reduction near boundaries
- **Precision mode** (25% speed multiplier)
- **Slew rate limiting** for smooth velocity changes

## Architecture

- **stepper_executor**: ISR-based step pulse generation (GPTimer, 40kHz)
- **motion_planner**: Generates motion segments from quintic trajectories
- **quintic**: Minimum-jerk polynomial interpolation
- **homing**: Endstop-based homing state machine
- **preset_storage**: NVS-based preset persistence
- **usb_serial**: Command parser for Raspberry Pi communication
- **motion_controller**: High-level coordination layer

## Quick Start

**New to this project?** See [QUICK_START.md](QUICK_START.md) for a fast getting-started guide.

## Building and Flashing

See [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) for detailed instructions.

**Quick Start:**

1. Install ESP-IDF v5.0 or later
2. Set ESP-IDF environment (run `export.bat` on Windows or `export.sh` on Linux/macOS)
3. Navigate to project directory: `cd camera_fysetc_e4`
4. Build: `idf.py build`
5. Flash: `idf.py -p COM3 flash monitor` (replace COM3 with your port)

**Windows users:** Use the provided `build_and_flash.bat` script:
```cmd
build_and_flash.bat COM3
```

**Linux/macOS users:** Use the provided `build_and_flash.sh` script:
```bash
chmod +x build_and_flash.sh
./build_and_flash.sh /dev/ttyUSB0
```

## Microstepping Configuration

The firmware automatically scales velocities and accelerations based on the microstepping configuration set in `board.h` via `MICROSTEP_SCALE`. 

**Important**: Velocities are specified in "full steps/sec" (not microsteps). The firmware automatically converts these to microsteps/sec based on your hardware configuration.

To configure microstepping, edit `board.h` and set `MICROSTEP_SCALE` to match your TMC2209 hardware configuration:
- `1.0f` = Full step (no microstepping)
- `2.0f` = Half step
- `4.0f` = Quarter step
- `8.0f` = Eighth step
- `16.0f` = Sixteenth step (default)
- `32.0f` = Thirty-second step

The microstepping is set via hardware pins (MS1, MS2) on the TMC2209 driver board. Check your board's documentation to determine the current microstepping setting.

## USB Serial Commands

Commands sent from Raspberry Pi over USB serial (115200 baud):

- `VEL <pan> <tilt> <zoom>` - Set velocities (full steps/sec) for manual mode
- `GOTO <n>` - Move to preset n using quintic interpolation
- `SAVE <n>` - Save current position as preset n
- `HOME` - Start sequential homing sequence for all axes (PAN, TILT, ZOOM)
  - PAN and TILT use physical endstops
  - ZOOM uses sensorless homing (TMC2209 stallGuard)
- `POS` - Query current positions (response: `POS:pan,tilt,zoom`)
- `STATUS` - Query system status
- `STOP` - Stop all motion immediately
- `PRECISION <0|1>` - Enable/disable precision mode (25% speed)
- `LIMITS <axis> <min> <max>` - Set soft limits for axis (PAN/TILT/ZOOM)

## Pin Configuration

See `board.h` for complete pin mapping. Key pins:

- PAN: Step=GPIO27, Dir=GPIO26, Endstop=GPIO34
- TILT: Step=GPIO33, Dir=GPIO32, Endstop=GPIO35
- ZOOM: Step=GPIO16, Dir=GPIO17
- Enable: GPIO25 (shared for all axes)

**Note**: GPIO34 and GPIO35 are input-only with no internal pullups. External pullups must be provided in your endstop wiring.

## Motion Profiles

The firmware uses **quintic polynomial interpolation** (minimum-jerk) for smooth motion between waypoints. Boundary conditions:
- Position at start/end: specified
- Velocity at start/end: 0
- Acceleration at start/end: 0

This produces film-quality smooth motion with no sudden acceleration changes.

Easing options:
- Linear
- Smootherstep (quintic smootherstep)
- Sigmoid/Logistic

## Presets

Presets are stored in ESP32 NVS and include:
- Target positions (pan, tilt, zoom)
- Easing type
- Duration or speed scale
- Arrival overshoot
- Approach mode (direct/home-first/safe-route)
- Per-preset speed/acceleration multipliers
- Precision mode preference

## References

- [FYSETC E4 Board](https://fysetc.github.io/E4/)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [GPTimer API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gptimer.html)

## License

See LICENSE file for details.

