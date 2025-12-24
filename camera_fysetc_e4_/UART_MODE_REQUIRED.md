# TMC2209 UART Mode Configuration Required

## Problem Confirmed

The diagnostics confirm that **TMC2209 register writes are not taking effect**. The driver is responding to UART reads but ignoring writes, which indicates it's **not in UART mode**.

## Evidence

From logs:
- ✅ UART communication works (valid responses with correct CRC)
- ✅ Write packets are being sent correctly
- ❌ Writes don't persist (GCONF still reads 0x00000000 after writing 0x00000444)
- ❌ SGTHRS still reads 0 after writing 150
- ❌ SG_RESULT always returns 0 (stallGuard not working)

## Root Cause

The TMC2209 driver is in **Standalone Mode** instead of **UART Mode**. In standalone mode:
- The driver responds to UART reads (returns default/reset values)
- But **ignores all UART writes**
- Configuration must be done via hardware (VREF potentiometer, etc.)

## Solution: Configure Hardware for UART Mode

The FYSETC E4 board needs to be configured to put the TMC2209 drivers into UART mode. This typically requires:

1. **PDN_UART Pin Configuration**
   - The PDN_UART pin on each TMC2209 determines the mode
   - Must be configured for UART mode (check board documentation)

2. **Board Jumpers**
   - FYSETC E4 may have jumpers to select UART vs Standalone mode
   - Check board documentation for jumper settings

3. **Board Documentation**
   - Refer to FYSETC E4 manual/schematic
   - Look for "UART mode" or "PDN_UART" configuration instructions

## Workaround Applied

Since sensorless homing isn't working without UART mode, I've **disabled zoom axis homing**:

- `HOME` command now only homes PAN and TILT (which have physical endstops)
- Zoom axis homing is skipped
- System will still function, but zoom won't be homed

## To Re-enable Sensorless Homing

Once you configure the FYSETC E4 board for UART mode:

1. **Check board documentation** for UART mode configuration
2. **Set jumpers/configure PDN_UART pins** as required
3. **Reboot the system**
4. **Check logs** - you should see:
   ```
   TMC2209 axis 2 GCONF immediately after write: 0x00000444
   ```
   (instead of 0x00000000)

5. **Re-enable sensorless homing** by changing in `homing.c`:
   ```c
   bool is_sensorless = (axis == AXIS_ZOOM);  // Re-enable this
   ```

## Current Status

- ✅ PAN and TILT homing: Working (physical endstops)
- ❌ ZOOM homing: Disabled (requires UART mode)
- ✅ Motor control: Working (zoom motor moves normally)
- ❌ Sensorless homing: Not available (requires UART mode)

The system will function normally, but zoom axis won't be homed automatically. You can manually position it or add a physical endstop if needed.

