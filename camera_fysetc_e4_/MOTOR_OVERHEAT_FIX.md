# Motor Overheating & Instant Homing Fix

## Problem
- Motor becomes extremely hot during homing
- SG_RESULT=0 (maximum stall) detected immediately
- Homing completes instantly

## Root Causes

1. **Motor current too high** - Default was IRUN=20 (out of 31), causing excessive heat
2. **Motor may be blocked** - SG_RESULT=0 indicates maximum load/stall
3. **No movement verification** - System doesn't verify motor is actually rotating

## Fixes Applied

### 1. Reduced Motor Current
Changed from:
- IHOLD=16, IRUN=20

To:
- IHOLD=8, IRUN=12

This reduces heat generation significantly. Current can be increased later if motor needs more torque.

### 2. Added Movement Diagnostics
- Logs movement progress during fast approach
- Shows SG_RESULT values as motor moves
- Warns if SG_RESULT=0 detected early (motor blocked)

### 3. Movement Tracking
- Logs actual movement distance when stall is detected
- Helps diagnose if motor is moving or stuck

## Additional Steps Needed

### Check Mechanical System
1. **Verify motor can rotate freely** - Manually rotate zoom mechanism
2. **Check for binding** - Ensure nothing is blocking the zoom travel
3. **Verify endstop position** - Make sure zoom can actually reach endstop

### Verify Step/Direction Signals
1. **Check step pulses** - Use oscilloscope to verify step pulses are being generated
2. **Check direction** - Verify direction pin is set correctly
3. **Check enable pin** - Motor should be enabled during homing

### Calibration Steps

1. **Test motor manually**:
   ```
   VEL 0 0 100  # Move zoom slowly
   # Monitor position and motor temperature
   ```

2. **If motor still overheats**:
   - Further reduce current: `tmc2209_set_current(AXIS_ZOOM, 6, 10)`
   - Check if motor driver is configured correctly
   - Verify motor voltage matches driver settings

3. **If SG_RESULT always 0**:
   - Motor may be physically blocked
   - Motor driver may not be configured correctly
   - Check TMC2209 configuration registers

### Expected Behavior After Fix

- Motor should run cooler (reduced current)
- Fast approach should show movement progress in logs
- If motor is blocked, logs will show early warning
- Movement distance will be logged when stall detected

### If Still Having Issues

1. **Disable sensorless homing temporarily** - Use physical endstop if available
2. **Check TMC2209 driver configuration** - Verify all registers are set correctly
3. **Test motor outside homing** - Use VEL command to test motor operation
4. **Check power supply** - Ensure adequate power and voltage

## Monitoring

Watch for these log messages:
- `Starting fast approach` - Homing started
- `Fast approach: movement=X, SG_RESULT=Y` - Progress updates
- `SG_RESULT=0 detected early` - Warning: motor may be blocked
- `Motor fully stalled (SG_RESULT=0)` - Stall detected after minimum movement

If you see SG_RESULT=0 immediately with little movement, the motor is likely blocked or not configured correctly.

