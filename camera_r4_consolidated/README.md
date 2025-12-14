# Arduino Uno R4 Minima - Consolidated Camera Motion Rig

This is a consolidated, optimized version of the camera motion rig that combines pan, tilt, and zoom control into a single Arduino Uno R4 Minima board.

## Key Improvements

### 1. **Non-Blocking Operation**
   - All stepper motor control uses non-blocking timers
   - Pitch and Yaw steppers use `micros()` for microsecond-precision timing
   - Zoom stepper uses `millis()` for millisecond timing (as per original design)
   - No `delay()` or `delayMicroseconds()` calls that block execution

### 2. **Consolidated Architecture**
   - All three stepper motors (pan, tilt, zoom) controlled from a single Arduino
   - Unified command interface compatible with existing Python controller
   - Single serial connection instead of two separate boards

### 3. **Optimized Zero Zoom**
   - `zero_zoom_pos()` is now completely non-blocking
   - Zeroing operation runs in the background without blocking other operations
   - Can still receive commands during zeroing (though zoom commands are blocked until zeroing completes)

### 4. **Endstop Zeroing on Boot**
   - Automatic zeroing of pitch and yaw axes using endstop switches on bootup
   - Non-blocking zeroing operations that run in the background
   - Automatically backs off from endstop after detection for accurate positioning
   - Zeroing completes before allowing setpoint motion for safety

### 5. **Persistent Preset Storage**
   - All preset positions are automatically saved to EEPROM
   - Presets are restored from EEPROM on bootup
   - No need to reprogram presets after power cycles
   - EEPROM includes magic number and version checking for data integrity

### 6. **Advanced Motion Control**
   - **Acceleration Ramping**: Smooth speed transitions prevent jerky movements
   - **Speed Interpolation**: Setpoint motion uses variable speed (fast start, slow end) for precision
   - **S-Curve Interpolation**: Optional smooth acceleration/deceleration curves for natural motion
   - **Soft Position Limits**: Configurable min/max limits prevent over-travel
   - **Endstop Monitoring**: Continuous endstop checking during operation for safety

### 7. **Safety Features**
   - **Emergency Stop**: Immediate halt of all movement via `estop` command
   - **Endstop Monitoring**: Real-time endstop checking prevents crashes
   - **Soft Limits**: Position limits prevent movement beyond safe ranges
   - **Position Offsets**: Calibration offsets for fine-tuning zero positions

### 8. **Configuration & Query Commands**
   - Position query commands to read current positions
   - Configuration commands for limits, offsets, and speeds
   - Status query for comprehensive system state
   - All settings persist in EEPROM

## Hardware Configuration

### Pin Assignments (CNC Shield with TMC2209)
- **Pitch (Tilt) - X-axis:**
  - Step: Pin 2
  - Direction: Pin 5
  
- **Yaw (Pan) - Y-axis:**
  - Step: Pin 3
  - Direction: Pin 6
  
- **Zoom - Z-axis:**
  - Step: Pin 4
  - Direction: Pin 7

- **Endstops (required for zeroing):**
  - EndstopX (Pitch/Tilt): Pin 9 (active LOW with INPUT_PULLUP)
  - EndstopY (Yaw/Pan): Pin 10 (active LOW with INPUT_PULLUP)
  
  **Note:** Endstops should be positioned at the minimum (zero) position for each axis. The system will automatically home to these positions on bootup.

## Serial Commands

All commands are space-delimited and compatible with the existing Python controller.

### Movement Commands
- `a` - Pitch forward
- `b` - Pitch reverse
- `c` - Pitch stop
- `1` - Yaw forward
- `2` - Yaw reverse
- `3` - Yaw stop
- `4` - Zoom out
- `5` - Zoom in
- `6` - Zoom stop

### Setpoint Commands
- `s` - Save current position as setpoint 1 (saved to EEPROM)
- `s2` - Save current position as setpoint 2 (saved to EEPROM)
- `s3` - Save current position as setpoint 3 (saved to EEPROM)
- `s4` - Save current position as setpoint 4 (saved to EEPROM)
- `t` - Move to setpoint 1
- `t2` - Move to setpoint 2
- `t3` - Move to setpoint 3
- `t4` - Move to setpoint 4

**Note:** All preset positions are automatically saved to EEPROM and will persist across reboots.

### Speed Control
- `p<value>` - Set pitch speed (microseconds per step)
- `y<value>` - Set yaw speed (microseconds per step)
- `z<value>` - Set zoom speed (milliseconds per step)

### Joystick Control (Proportional)
- `j,yaw,pitch,zoom` - Send joystick values (-32768 to 32768)
  - Arduino interprets joystick positions into movement direction and speed
  - Half-stick input = half speed for fine control
  - Example: `j,16384,0,-16384` = half yaw right, no pitch, half zoom out

### Utility Commands
- `info` - Returns "consolidated_module"
- `ea` - Reset zoom position to 0
- `eb` - Update zoom B stop position
- `flush` - Flush serial input buffer

### Emergency Stop
- `estop` - Immediately stop all movement and set emergency stop flag
- `estop_clear` - Clear emergency stop flag and resume normal operation

### Position Query Commands
- `pos` - Returns current positions: `POS:pitch,yaw,zoom`
  - Positions include offset calibration
- `status` - Returns comprehensive status: `STATUS:P,pos,M,move,S,speed Y,pos,M,move,S,speed Z,pos,M,move,S,speed ESTOP,flag SETPOINT,num`
  - Includes position, movement direction, speed, emergency stop state, and active setpoint

### Configuration Commands
- `plim,min,max` - Set pitch soft limits (e.g., `plim,-10000,10000`)
- `ylim,min,max` - Set yaw soft limits (e.g., `ylim,-20000,20000`)
- `zlim,min,max` - Set zoom soft limits (e.g., `zlim,0,2000`)
- `limits` - Query all soft limits: `LIMITS:P,min,max Y,min,max Z,min,max`

### Position Offset/Calibration
- `poff,value` - Set pitch position offset (calibration adjustment)
- `yoff,value` - Set yaw position offset
- `zoff,value` - Set zoom position offset
- `offsets` - Query all offsets: `OFFSETS:P,value Y,value Z,value`
- **Note**: Offsets are applied when reporting positions and when saving/loading presets

### Movement Interpolation
- `smooth,0` - Disable smooth S-curve interpolation (linear speed)
- `smooth,1` - Enable smooth S-curve interpolation (default)
  - Provides smooth acceleration and deceleration curves
  - Makes motion more natural and reduces mechanical stress

## Performance Improvements

1. **Reduced Latency**: Non-blocking timers allow the system to respond to serial commands immediately, even during stepper movement
2. **Smoother Operation**: All three axes can move simultaneously without interference
3. **Better Responsiveness**: Serial command processing happens every loop iteration, not blocked by stepper delays

## Technical Details

### Timing Precision
- **Pitch/Yaw**: Uses `micros()` for microsecond-level precision (required for high-speed stepper control)
- **Zoom**: Uses `millis()` for millisecond-level precision (sufficient for zoom control)

### Step Pulse Generation
All steppers use toggle-based step generation:
```cpp
digitalWrite(StepPin, !digitalRead(StepPin));
```
This creates a clean pulse train without needing separate HIGH/LOW delays.

### State Management
- Each stepper maintains its own position counter
- Setpoint motion coordinates all three axes simultaneously
- Zero zoom operation runs as a background state machine
- Endstop zeroing runs automatically on bootup for pitch and yaw axes

### Endstop Zeroing Behavior
- On bootup, pitch and yaw axes automatically move towards their endstops
- When endstop is triggered (LOW), the axis backs off 100 steps
- Position is then set to zero (EndstopDefaultPos)
- Zeroing must complete before setpoint motion is allowed (safety feature)
- Zeroing speed is set to 2000 microseconds per step for accuracy
- **Timeout Protection**: 
  - Pitch (tilt down) will timeout after 800 steps (90 degrees maximum)
  - Yaw (pan) will timeout after 1600 steps (180 degrees maximum)
  - If timeout occurs, an error message is sent via Serial and manual control is enabled
  - Timeout values are configurable via `MAX_PITCH_ZERO_STEPS` and `MAX_YAW_ZERO_STEPS` constants

### EEPROM Storage
- Preset positions, limits, and offsets stored in EEPROM with magic number validation
- Memory layout:
  - Address 0-1: Magic number (0xCAFE) for data validation
  - Address 2: Version number (currently 2)
  - Address 3-26: 4 presets × 6 bytes each (2 bytes per axis: pitch, yaw, zoom)
  - Address 27-38: Soft limits (6 limits × 2 bytes: pitch min/max, yaw min/max, zoom min/max)
  - Address 39-44: Position offsets (3 offsets × 2 bytes: pitch, yaw, zoom)
- If EEPROM data is invalid or missing, defaults to zero positions and default limits
- All configuration (presets, limits, offsets) is automatically saved to EEPROM when changed

## Dependencies

- **SafeStringReader**: Required for serial command parsing
  - Install via Arduino Library Manager: "SafeString"

## Advanced Features

### Acceleration Ramping
- Speed changes are gradually ramped to prevent sudden jerks
- Acceleration step: 50 microseconds per update interval
- Update interval: 1000 microseconds
- Applies to pitch and yaw axes during joystick control
- Provides smooth transitions when changing speed or direction

### Speed Interpolation for Setpoint Motion
- Variable speed based on distance to target:
  - **Fast zone** (>500 steps): Full speed for rapid movement
  - **Slow zone** (<50 steps): 30% speed for precision positioning
  - **Transition zone**: Smooth interpolation between fast and slow
- Optional S-curve easing for even smoother motion
- Reduces overshoot and improves positioning accuracy

### Soft Position Limits
- Configurable min/max limits for each axis
- Movement is automatically stopped at limits
- Limits are stored in EEPROM and persist across reboots
- Prevents over-travel and protects hardware
- Default limits:
  - Pitch: -10000 to 10000
  - Yaw: -20000 to 20000
  - Zoom: 0 to 2000

### Endstop Monitoring
- Continuous monitoring of endstop switches during normal operation
- Automatically stops movement if endstop is triggered unexpectedly
- Safety feature to prevent crashes if limits are exceeded
- Only active during normal operation (not during zeroing)

### Position Offset/Calibration
- Fine-tune zero positions without physical adjustment
- Offsets are applied when:
  - Reporting positions via `pos` or `status` commands
  - Saving/loading presets
- Useful for calibration after mechanical adjustments
- Offsets are stored in EEPROM

## Notes

- **Bootup Sequence:**
  1. EEPROM presets, limits, and offsets are loaded
  2. Pitch and yaw axes begin endstop zeroing (non-blocking)
  3. Zoom begins zeroing sequence (non-blocking)
  4. System is ready once all zeroing operations complete
  
- All stepper speeds are configurable via serial commands
- The system maintains backward compatibility with existing command protocols
- Endstops must be properly connected and positioned for accurate zeroing
- **Timeout Protection**: If endstops are not reached within the maximum step limits, zeroing will timeout and report an error
  - This prevents the camera from moving too far if an endstop fails or is mispositioned
  - After timeout, manual control is enabled so you can troubleshoot
- Preset positions, limits, and offsets are automatically saved to EEPROM - no manual save required
- **Configuring Step Limits**: Adjust `MAX_PITCH_ZERO_STEPS` and `MAX_YAW_ZERO_STEPS` in the code based on your stepper motor configuration:
  - Typical: 200 steps/rev with 16x microstepping = 3200 steps/rev
  - 90 degrees = 800 steps, 180 degrees = 1600 steps
  - Adjust if using different microstepping or gear ratios
- **Emergency Stop**: Use `estop` command to immediately halt all movement in case of emergency
- **Debug Output**: Debug messages can be disabled by setting `ENABLE_DEBUG_OUTPUT` to `false` in code

