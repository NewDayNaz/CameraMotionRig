# PTZ Controller for Arduino Uno R4 Minima

Film-quality 3-axis cinematic PTZ (Pan-Tilt-Zoom) controller firmware for Arduino Uno R4 Minima with CNC Shield.

## Features

- **Film-Quality Smooth Motion**: Quintic polynomial interpolation for minimum-jerk trajectories
- **Deterministic Step Execution**: Segment-based execution (5-10ms segments) with low jitter
- **Endstop Homing**: Automatic homing for PAN and TILT axes using mechanical endstops
- **Preset Storage**: Save and recall camera positions with cinematic parameters (easing, duration, overshoot)
- **Manual Joystick Control**: Velocity control via USB serial from Raspberry Pi with:
  - Deadband filtering (5% default)
  - Exponential curve (1.8 default)
  - Slew rate limiting (acceleration limiting)
  - Soft limit zones (gradual speed reduction near limits)
  - Precision mode (25% speed multiplier)
- **USB Serial Protocol**: Simple text-based command protocol for Raspberry Pi integration

## Hardware Requirements

- Arduino Uno R4 Minima
- CNC Shield for Arduino (compatible with TMC2209 stepper drivers)
- 3x Stepper motors (Pan, Tilt, Zoom)
- 2x Mechanical endstops (for Pan and Tilt, active LOW)
- TMC2209 stepper drivers (or compatible)

## Pin Mapping (CNC Shield)

- **PAN (X-axis)**:
  - Step: Pin 2
  - Dir: Pin 5
  - Endstop: Pin 9 (MIN, active LOW)

- **TILT (Y-axis)**:
  - Step: Pin 3
  - Dir: Pin 6
  - Endstop: Pin 10 (MIN, active LOW)

- **ZOOM (Z-axis)**:
  - Step: Pin 4
  - Dir: Pin 7
  - Endstop: None (optional)

- **Enable**: Pin 8 (shared for all axes)

## Installation

1. Open `ptz_controller_r4.ino` in Arduino IDE
2. Select board: **Arduino Uno R4 Minima**
3. Upload to board

## USB Serial Command Protocol

Commands are sent as text over USB serial at 115200 baud. All commands are case-insensitive.

### Motion Commands

- `VEL <pan> <tilt> <zoom>` - Set velocities (normalized -1.0 to 1.0)
  - Example: `VEL 0.5 -0.3 0.0`
  - Enters manual velocity mode
  - Velocities are filtered with deadband, expo, slew limiting, and soft limits

- `GOTO <n>` - Move to preset n using quintic interpolation
  - Example: `GOTO 0`
  - Exits manual mode and plans smooth move to preset position

- `HOME [axis]` - Start homing sequence
  - Example: `HOME` or `HOME PAN` or `HOME TILT`
  - Performs fast approach, backoff, then slow approach to endstop

- `STOP` - Emergency stop (stops all motion)

### Preset Commands

- `SAVE <n>` - Save current position as preset n (0-15)
  - Example: `SAVE 0`
  - Saves current pan/tilt/zoom positions to EEPROM

- `GOTO <n>` - Move to preset n
  - Uses quintic interpolation with default easing

### System Commands

- `PRECISION <0|1>` - Enable/disable precision mode
  - Example: `PRECISION 1`
  - Precision mode reduces speeds to 25% for fine control

- `POS` - Get current positions
  - Returns: `POS <pan> <tilt> <zoom>`

- `STATUS` - Get system status
  - Returns: `STATUS <busy|idle> <homing|ready> <precision>`

### Response Format

- `OK` - Command succeeded
- `ERROR: <message>` - Command failed with error message
- `STATUS: <message>` - Status message

## Motion System Architecture

### Segment-Based Execution

Motion is divided into fixed-duration segments (8ms default). Each segment contains:
- Step counts for each axis (signed integers)
- Duration in microseconds

The stepper executor consumes segments from a ring buffer queue and distributes steps evenly across the segment duration using a DDA/Bresenham accumulator.

### Quintic Polynomial Interpolation

Waypoint moves use quintic polynomial interpolation for smooth, film-quality motion:

```
x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
```

Boundary conditions:
- Position at start/end: specified
- Velocity at start/end: 0
- Acceleration at start/end: 0

This produces minimum-jerk trajectories ideal for cinematic camera movement.

### Easing Functions

Three easing types are supported:
- `LINEAR` - No easing
- `SMOOTHERSTEP` - Quintic smootherstep (default)
- `SIGMOID` - Sigmoid/logistic curve

### Manual Velocity Mode

When receiving `VEL` commands, the system:
1. Applies deadband (5% default) - ignores small joystick movements
2. Applies exponential curve (1.8 default) - makes small movements more precise
3. Applies slew rate limiting - limits acceleration to 1/2 of max acceleration
4. Applies soft limit zones - gradually reduces speed in last 5% of travel
5. Applies precision mode multiplier - 25% speed if enabled

## Configuration

### Motion Limits

Edit in `ptz_controller_r4.ino` setup():
```cpp
motion_planner_set_limits(&g_motion_planner, AXIS_PAN, -20000.0f, 20000.0f);
motion_planner_set_limits(&g_motion_planner, AXIS_TILT, -10000.0f, 10000.0f);
motion_planner_set_limits(&g_motion_planner, AXIS_ZOOM, 0.0f, 2000.0f);
```

### Max Velocities

Edit in `motion_planner.h`:
```cpp
#define MAX_VELOCITY_PAN  2000.0f  // steps/sec
#define MAX_VELOCITY_TILT 2000.0f
#define MAX_VELOCITY_ZOOM 1500.0f
```

### Max Accelerations

Edit in `motion_planner.h`:
```cpp
#define MAX_ACCEL_PAN  1000.0f  // steps/sec^2
#define MAX_ACCEL_TILT 1000.0f
#define MAX_ACCEL_ZOOM 800.0f
```

### Joystick Filtering

Edit in `motion_planner.h`:
```cpp
#define JOYSTICK_DEADBAND 0.05f  // 5% deadband
#define JOYSTICK_EXPO 1.8f       // Exponential curve factor
```

## Homing Sequence

1. **Fast Approach**: Move toward min endstop at 1000 steps/sec
2. **Backoff**: Move away from endstop by 500 steps
3. **Slow Approach**: Move toward endstop at 200 steps/sec
4. **Set Home**: When endstop triggers, set position to 0

Homing can be started with `HOME` command. Only PAN and TILT axes have endstops.

## Preset Storage

Presets are stored in EEPROM and persist across power cycles. Each preset stores:
- Position (pan, tilt, zoom)
- Easing type
- Duration (0 = auto-calculate)
- Speed scale (0 = use duration)
- Arrival overshoot
- Approach mode
- Speed/accel multipliers
- Precision preference

Up to 16 presets can be stored (indices 0-15).

## Raspberry Pi Integration

The Raspberry Pi should:
1. Read joystick input
2. Send `VEL` commands over USB serial
3. Send `GOTO`, `SAVE`, `HOME` commands as needed
4. Monitor `STATUS` for system state

Example Python code:
```python
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)

# Send velocity command
ser.write(b"VEL 0.5 -0.3 0.0\n")

# Read response
response = ser.readline()
print(response.decode())
```

## File Structure

- `ptz_controller_r4.ino` - Main firmware entry point
- `board.h/cpp` - Pin definitions and GPIO control
- `segment.h/cpp` - Motion segment queue (ring buffer)
- `quintic.h/cpp` - Quintic polynomial interpolation
- `stepper_executor.h/cpp` - Deterministic step generation
- `motion_planner.h/cpp` - Motion planning and segment generation
- `homing.h/cpp` - Endstop-based homing
- `preset_storage.h/cpp` - EEPROM preset storage
- `usb_serial.h/cpp` - USB serial command parser

## Troubleshooting

### Motors not moving
- Check enable pin (should be LOW to enable)
- Verify step/dir pin connections
- Check stepper driver power and configuration

### Homing fails
- Verify endstop wiring (active LOW with pullup)
- Check endstop switch operation
- Increase timeout in `homing.cpp` if needed

### Motion is jerky
- Reduce max velocities
- Increase segment duration
- Check for mechanical binding

### Serial communication issues
- Verify baud rate (115200)
- Check USB cable connection
- Ensure commands end with newline (`\n`)

## License

This firmware is provided as-is for use with Arduino Uno R4 Minima and CNC Shield.

