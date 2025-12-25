# StallGuard Always Reading 0 - Diagnostics Added

## Problem Observed

During homing, `SG_RESULT=0` is being read **constantly**, even when the motor is clearly moving (position is changing). This indicates a problem with stallGuard configuration or communication.

## What SG_RESULT=0 Means

- **SG_RESULT=0**: Maximum load/stall detected (motor fully blocked)
- **SG_RESULT=255**: Invalid/no reading (motor not moving fast enough, or stallGuard disabled)
- **Normal range**: 50-200 when motor is free-running

## Possible Causes

1. **stallGuard not properly enabled** - GCONF bit 2 may not be set correctly
2. **UART communication issue** - Register reads may be failing/returning wrong values
3. **TCOOLTHRS too high** - Motor may not be moving fast enough for valid readings
4. **Motor actually stalling** - If motor is overloaded, it could be stalling (but unlikely if position is changing)
5. **VREF too low** - Motor might not have enough current to run properly

## Diagnostics Added

### 1. Enhanced Initialization Logging
- Now logs GCONF register value after configuration
- Verifies stallGuard enable bit is set
- Logs SGTHRS value
- Reads initial SG_RESULT to check if it's working

### 2. Periodic Diagnostic Logging
- Logs position, movement, and SG_RESULT every 0.5 seconds during homing
- Helps identify if stallGuard is working at all

### 3. SG_RESULT Read Logging
- Logs actual SG_RESULT reads from TMC2209 (every 100th read to avoid spam)
- Helps verify UART communication is working

## What to Check in Logs

After flashing, look for these log messages on startup:

1. **GCONF verification**:
   ```
   TMC2209 axis 2 GCONF verify: 0xXXXXXXXX (stallGuard enabled/DISABLED!)
   ```
   - Should show "enabled", not "DISABLED!"

2. **SGTHRS verification**:
   ```
   TMC2209 axis 2 SGTHRS verify: 150
   ```
   - Should show 150 (or whatever threshold is set)

3. **Initial SG_RESULT**:
   ```
   TMC2209 axis 2 initial SG_RESULT: XXX
   ```
   - Should NOT be 0 (unless motor is actually stalled)
   - Should NOT be 255 (unless motor isn't moving)

4. **During homing**:
   ```
   Sensorless homing: pos=XXX, movement=XXX, SG_RESULT=XXX
   ```
   - Watch if SG_RESULT changes at all
   - If it's always 0, stallGuard isn't working

## Troubleshooting Steps

### If SG_RESULT is always 0:

1. **Check GCONF register** - Verify bit 2 (stallGuard enable) is set
2. **Check UART communication** - Verify register reads are working
3. **Check TCOOLTHRS** - Currently set to 30, motor should be moving above this
4. **Check motor current** - IRUN=12 might be too low, try increasing to 16-18
5. **Check VREF potentiometer** - May be set too low on the board

### If SG_RESULT is always 255:

- Motor isn't moving fast enough for stallGuard to work
- TCOOLTHRS might be too high (but it's set to 30, which is low)
- Motor might not be moving at all (mechanical issue)

### If SG_RESULT varies but always reads 0 in code:

- UART communication might be returning wrong values
- Register read function might have a bug
- Byte order/parsing issue

## Next Steps

1. Flash the updated firmware
2. Check startup logs for configuration verification
3. Watch diagnostic logs during homing
4. Report what SG_RESULT values you see
5. Based on results, we can adjust configuration or fix communication issues


