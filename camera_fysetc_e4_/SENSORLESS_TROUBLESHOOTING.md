# Sensorless Homing Troubleshooting Guide

## Problem: Homing Completes Instantly

If homing completes too quickly (within milliseconds), the stallGuard is likely detecting a false stall. Here are the fixes applied and additional steps:

### Fixes Applied

1. **Lowered TCOOLTHRS**: Changed from 500 to 30 steps/sec
   - **Why**: Slow homing speed (50 steps/sec) was below TCOOLTHRS, so stallGuard didn't work during slow approach
   - **Now**: TCOOLTHRS (30) < Slow speed (50), so stallGuard works at all homing speeds

2. **Added Minimum Movement Check**: Requires 50 steps of movement before checking for stall
   - **Why**: Prevents false triggers when motor first starts (motor may appear stalled before it starts moving)
   - **Effect**: Motor must move at least 50 steps before stall detection is enabled

3. **Improved Logging**: Now shows SG_RESULT values when stall is detected
   - **Why**: Helps diagnose threshold calibration issues
   - **Use**: Check logs to see actual SG_RESULT values

4. **Ignore Invalid Readings**: SG_RESULT=255 (invalid) is now ignored
   - **Why**: 255 typically means motor isn't moving fast enough for valid reading

### Additional Troubleshooting Steps

#### 1. Check Actual SG_RESULT Values

Add temporary logging to see what values you're getting:

```c
// In homing_check_stall(), add:
ESP_LOGI(TAG, "SG_RESULT=%d, threshold=%d, movement=%.1f", 
         sg_result, homing_status.stall_threshold, 
         fabsf(current_position - homing_status.stall_check_start_pos));
```

#### 2. Adjust StallGuard Threshold

The default threshold (150) may be too sensitive. Try:

- **If still too sensitive**: Increase threshold to 180-200
- **If not sensitive enough**: Decrease threshold to 100-120

You can adjust at runtime (if you add a command) or change `TMC2209_DEFAULT_SGTHRS` in `tmc2209.h`.

#### 3. Verify Motor is Actually Moving

Check if the motor is moving during homing:
- Watch the zoom mechanism physically move
- Check position logs - position should change during homing
- If position doesn't change, motor isn't moving (mechanical issue or driver problem)

#### 4. Check Motor Current

If motor current is too low, stallGuard may not work reliably:
- Current is set to IHOLD=16, IRUN=20 (out of 31 max)
- If motor is weak or has high load, increase current:
  ```c
  tmc2209_set_current(AXIS_ZOOM, 20, 24);  // Higher current
  ```

#### 5. Verify TCOOLTHRS is Applied

Check logs on startup - you should see:
```
TMC2209 axis 2 TCOOLTHRS: 30
```

If it still shows 500, the register write may have failed.

#### 6. Test StallGuard Reading Manually

Add a test command to read SG_RESULT while motor moves:
- Move motor manually with `VEL` command
- Read SG_RESULT values
- Block motor manually and see if SG_RESULT drops

### Expected Behavior

**Normal homing sequence:**
1. Fast approach: Motor moves at 500 steps/sec toward endstop
2. After ~50 steps of movement, stall detection becomes active
3. When motor hits endstop, SG_RESULT drops below threshold
4. After 3 consecutive readings, stall is confirmed
5. Backoff: Motor moves away 200 steps
6. Slow approach: Motor moves at 50 steps/sec (above TCOOLTHRS of 30)
7. Stall detected again, homing complete

**If homing completes instantly:**
- Check if motor is actually moving (position should change)
- Check SG_RESULT values in logs
- Verify TCOOLTHRS is set correctly
- Try increasing threshold if too sensitive

### Calibration Procedure

1. **Start with default threshold (150)**
2. **Run homing sequence**
3. **Check logs for SG_RESULT values:**
   - When motor is free-running: Should be > 150
   - When motor is blocked: Should be < 150
4. **Adjust threshold:**
   - If false triggers: Increase threshold
   - If doesn't detect: Decrease threshold
5. **Repeat until reliable**

### Common Issues

**Issue**: Homing completes in < 100ms
- **Cause**: Threshold too sensitive or motor not moving
- **Fix**: Increase threshold, check motor movement

**Issue**: Homing never completes (timeout)
- **Cause**: Threshold not sensitive enough or motor current too low
- **Fix**: Decrease threshold, increase motor current

**Issue**: Inconsistent detection
- **Cause**: Electrical noise or unstable motor current
- **Fix**: Increase debounce count, check wiring, stabilize power supply

