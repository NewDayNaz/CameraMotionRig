# TMC2209 UART Communication Issue

## Problem

All TMC2209 registers are reading back as `0x00000000` after writes, indicating that:
1. Register writes are not taking effect, OR
2. UART communication is not working correctly, OR
3. The TMC2209 is not responding (not in UART mode, wrong address, or not powered)

## Symptoms

From logs:
```
I (472) tmc2209: TMC2209 axis 2 GCONF: 0x00000000 (verified)
I (536) tmc2209: TMC2209 axis 2 SGTHRS verify: 0
I (538) tmc2209: TMC2209 axis 2 initial SG_RESULT: 0
```

All registers read as 0, which suggests UART communication is failing.

## Possible Causes

### 1. TMC2209 Not in UART Mode
- The driver might be in step/direction mode by default
- Needs to be configured for UART mode via hardware jumpers or register writes

### 2. Wrong UART Address
- If multiple drivers share UART bus, they need different addresses
- FYSETC E4 board may have specific addressing scheme

### 3. UART Wiring Issues
- TX/RX pins might be swapped
- No pullup resistors on UART lines
- Wrong baud rate (currently 115200)

### 4. Driver Not Powered/Enabled
- TMC2209 needs power
- Enable pin might need to be set

### 5. UART Protocol Issue
- CRC calculation might be wrong
- Packet format might be incorrect
- Timing might be off

## Diagnostics Added

1. **Enhanced logging** - Shows raw UART response bytes
2. **CRC verification logging** - Shows if CRC mismatches occur
3. **Initial GCONF read** - Checks state before writing
4. **GSTAT verification** - Confirms basic communication works

## Next Steps to Debug

1. **Check UART wiring**:
   - Verify GPIO22 (TX) and GPIO21 (RX) connections
   - Check if pullups are needed
   - Verify baud rate matches

2. **Check FYSETC E4 documentation**:
   - How are TMC2209 drivers configured for UART mode?
   - Are there jumpers that need to be set?
   - What is the addressing scheme?

3. **Test with oscilloscope/logic analyzer**:
   - Verify UART packets are being sent
   - Check if TMC2209 is responding
   - Verify timing

4. **Try reading different registers**:
   - Some registers might be readable even if writes fail
   - GSTAT (0x01) should always be readable

5. **Check if driver responds at all**:
   - If all reads return 0, driver might not be responding
   - Could indicate power/wiring issue

## Expected Behavior

After fixes, you should see:
- GSTAT reading non-zero (even if all zeros, indicates communication works)
- GCONF reading back written value
- SGTHRS reading back written value (150)
- SG_RESULT reading non-zero when motor is running

If all registers still read 0, UART communication is fundamentally broken and needs hardware/configuration fixes.


