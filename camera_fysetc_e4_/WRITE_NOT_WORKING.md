# TMC2209 Register Writes Not Taking Effect

## Problem

TMC2209 is responding to UART reads (communication works), but register writes are not taking effect. All registers read back as 0x00000000.

## Evidence

From logs:
- UART communication works: responses have correct sync bytes (0x05) and valid CRC
- Reads succeed: Getting 8-byte responses
- Writes appear to send: No write errors reported
- But writes don't persist: All registers read back as 0x00000000

Raw response example:
```
I (542) tmc2209: TMC2209 read response (addr=0x01): 05 01 00 00 00 00 40 00
I (549) tmc2209: TMC2209 read response (addr=0x00): 05 00 00 00 00 00 D7 00
```

The TMC2209 IS responding, but data bytes (2-5) are all zeros.

## Possible Causes

### 1. TMC2209 Not in UART Mode (Most Likely)
The TMC2209 needs to be in UART mode for register writes to work. This is typically controlled by:
- **PDN_UART pin**: Must be configured for UART mode (usually pulled LOW or connected to enable UART)
- **Hardware jumpers**: FYSETC E4 board may have jumpers to enable UART mode
- **GCONF bit 6 (UART_EN)**: Software can try to enable it, but hardware must support it first

### 2. Write Protection
Some TMC2209 configurations may have write protection enabled (unlikely for default config)

### 3. Timing Issues
Writes might need more time to be processed/committed (we added delays)

### 4. Multiple Drivers on Bus
If multiple TMC2209s share the UART bus, addressing might be wrong (but reads work, so this is less likely)

## What to Check

### Hardware Configuration
1. **FYSETC E4 Board Documentation**:
   - How to enable UART mode for TMC2209 drivers?
   - Are there jumpers to set?
   - Is PDN_UART pin configured correctly?

2. **Check Board Jumpers**:
   - Look for UART/Standalone mode jumpers
   - Verify jumpers are set for UART mode

3. **Check PDN_UART Pin**:
   - This pin determines if driver is in UART or standalone mode
   - Should be configured for UART (consult board docs)

### Software Attempts
We've tried:
- Setting GCONF bit 6 (UART_EN) via software
- Adding delays after writes
- Verifying write packets are sent correctly

But if hardware isn't configured for UART mode, software writes won't work.

## Next Steps

1. **Check FYSETC E4 documentation** for UART mode configuration
2. **Inspect board** for jumpers or configuration options
3. **Test with other axes** - try initializing axis 0 (PAN) or axis 1 (TILT) to see if behavior is the same
4. **Check if reads work for other registers** - try reading IHOLD_IRUN, TSTEP, etc. to see if they're also zero

## Diagnostic Output

The new code will:
- Log write packets being sent
- Read back registers immediately after writing
- Compare expected vs actual values
- Report if writes are taking effect

Look for these messages after flashing:
```
TMC2209 axis 2 writing GCONF: 0x00000444
TMC2209 axis 2 GCONF immediately after write: 0x00000000
TMC2209 axis 2 GCONF write did not take effect
```

This will confirm if writes are being ignored by the driver.


