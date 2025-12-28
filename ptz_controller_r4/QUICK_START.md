# Quick Start Guide

## Hardware Setup

1. **CNC Shield Installation**
   - Mount CNC Shield on Arduino Uno R4 Minima
   - Install TMC2209 stepper drivers in X, Y, Z slots
   - Connect stepper motors:
     - PAN → X-axis
     - TILT → Y-axis
     - ZOOM → Z-axis

2. **Endstop Wiring**
   - Connect PAN endstop to pin 9 (MIN, active LOW)
   - Connect TILT endstop to pin 10 (MIN, active LOW)
   - Use INPUT_PULLUP (no external pullup needed)

3. **Power**
   - Connect stepper motor power supply to CNC Shield
   - Ensure voltage matches your stepper motors (typically 12V or 24V)

## Software Setup

1. **Install Arduino IDE**
   - Download Arduino IDE 2.x or later
   - Install Arduino Uno R4 Minima board support

2. **Upload Firmware**
   - Open `ptz_controller_r4/ptz_controller_r4.ino`
   - Select board: **Arduino Uno R4 Minima**
   - Select port
   - Click Upload

3. **Open Serial Monitor**
   - Set baud rate to 115200
   - You should see: `PTZ Controller Ready`

## First Steps

### 1. Home the System

```
HOME PAN
HOME TILT
```

Wait for `OK` response. The system will:
- Fast approach to endstop
- Back off 500 steps
- Slow approach to endstop
- Set position to 0

### 2. Test Manual Control

```
VEL 0.5 0.0 0.0
```

This moves PAN at 50% speed. Send `VEL 0.0 0.0 0.0` to stop.

### 3. Save a Preset

Move to desired position, then:
```
SAVE 0
```

### 4. Recall a Preset

```
GOTO 0
```

The system will smoothly move to the saved position using quintic interpolation.

## Command Reference

| Command | Description | Example |
|---------|-------------|---------|
| `VEL <p> <t> <z>` | Set velocities (-1.0 to 1.0) | `VEL 0.5 -0.3 0.0` |
| `GOTO <n>` | Move to preset n | `GOTO 0` |
| `SAVE <n>` | Save current position as preset | `SAVE 0` |
| `HOME [axis]` | Home PAN or TILT | `HOME PAN` |
| `STOP` | Emergency stop | `STOP` |
| `PRECISION <0\|1>` | Toggle precision mode | `PRECISION 1` |
| `POS` | Get current positions | `POS` |
| `STATUS` | Get system status | `STATUS` |

## Troubleshooting

**Motors don't move:**
- Check enable pin (should be LOW to enable)
- Verify step/dir connections
- Check stepper driver power

**Homing fails:**
- Verify endstop wiring (active LOW)
- Test endstop with multimeter
- Check endstop switch operation

**Serial not working:**
- Verify baud rate (115200)
- Check USB cable
- Ensure commands end with newline

## Configuration

Edit limits in `ptz_controller_r4.ino`:
```cpp
motion_planner_set_limits(&g_motion_planner, AXIS_PAN, -20000.0f, 20000.0f);
```

Edit speeds in `motion_planner.h`:
```cpp
#define MAX_VELOCITY_PAN  2000.0f  // steps/sec
```

## Next Steps

- Calibrate soft limits for your mechanical setup
- Tune max velocities and accelerations
- Create presets for common camera positions
- Integrate with Raspberry Pi joystick controller

