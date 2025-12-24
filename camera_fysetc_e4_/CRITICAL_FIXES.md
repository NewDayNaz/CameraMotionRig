# Critical Fixes for Motor Overheating

## Changes Made

### 1. Reduced Motor Current (CRITICAL)
- **Before**: IHOLD=16, IRUN=20 (64% max current)
- **After**: IHOLD=8, IRUN=12 (39% max current)
- **Why**: Motor was running too hot, causing excessive heat generation

### 2. Added Safety Abort
- If SG_RESULT=0 (fully stalled) for 2+ seconds without significant movement, homing aborts
- Prevents motor from being driven continuously when blocked
- Protects motor and driver from damage

### 3. Enhanced Diagnostics
- Logs movement progress every 100 steps
- Shows SG_RESULT values during movement
- Warns when motor appears blocked

## Important: Check Your Hardware

Before running homing again, please verify:

1. **Motor can rotate freely** - Manually turn the zoom mechanism
2. **Nothing is blocking travel** - Check for mechanical binding
3. **Motor wiring** - Verify all connections are correct
4. **Power supply** - Ensure adequate current capacity

## If Motor Still Overheats

1. **Further reduce current** in `tmc2209.c`:
   ```c
   uint32_t ihold_irun = (6 << 0) | (10 << 8) | (5 << 16);  // Even lower
   ```

2. **Check if motor is physically blocked** - SG_RESULT=0 indicates maximum load

3. **Verify TMC2209 driver configuration** - May need to check board jumpers/settings

4. **Consider using physical endstop** instead of sensorless if issues persist

## Testing After Fix

1. **Let motor cool down** before testing again
2. **Test manual movement first**: `VEL 0 0 50` (low speed)
3. **Monitor temperature** during test
4. **Check logs** for movement progress and SG_RESULT values

If motor still gets hot during manual movement, the issue is not with homing code but with motor/driver configuration.

