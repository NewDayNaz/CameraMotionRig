# Sensorless Homing for Zoom Axis (TMC2209 stallGuard)

This document describes what's needed to add sensorless homing to the zoom axis using TMC2209's stallGuard feature.

## Overview

Sensorless homing uses the TMC2209 driver's stallGuard feature to detect when the motor stalls (hits a mechanical endstop) by monitoring the motor's back-EMF. This eliminates the need for a physical endstop switch on the zoom axis.

## Hardware Requirements

1. **TMC2209 drivers** with UART connectivity (already present on FYSETC E4)
2. **UART pins**: GPIO22 (TX), GPIO21 (RX) for communication
3. **Zoom axis motor**: Connected to E0 socket (GPIO16 step, GPIO17 dir)

## What Needs to Be Implemented

### 1. TMC2209 UART Driver Interface

Create a new module `tmc2209.h/c` to handle:
- UART communication with TMC2209 drivers
- Reading/writing TMC2209 registers
- Configuring stallGuard parameters
- Reading stallGuard value (SG_RESULT register)

**Key TMC2209 Registers:**
- `GCONF` (0x00) - Enable stallGuard
- `IHOLD_IRUN` (0x10) - Set motor current
- `TPOWERDOWN` (0x11) - Power down time
- `TSTEP` (0x12) - Actual step counter
- `TPWMTHRS` (0x13) - Upper velocity threshold
- `TCOOLTHRS` (0x14) - Lower velocity threshold for stallGuard
- `SGTHRS` (0x40) - stallGuard threshold
- `SG_RESULT` (0x41) - stallGuard result (0-255, lower = more load)
- `COOLCONF` (0x42) - CoolStep and stallGuard configuration

### 2. TMC2209 Initialization

During system startup, configure the zoom axis TMC2209 driver:
- Enable UART mode
- Set motor current (IHOLD, IRUN)
- Enable stallGuard
- Set stallGuard threshold (SGTHRS) - this is critical and needs tuning
- Set TCOOLTHRS (minimum velocity for stallGuard to work)

### 3. Stall Detection Logic

During sensorless homing:
- Move motor slowly toward endstop
- Continuously read SG_RESULT register
- Detect stall when SG_RESULT drops below threshold (motor is loaded/blocked)
- The threshold value needs to be calibrated for your specific motor/load

### 4. Update Homing System

Modify `homing.h/c` to support:
- Sensorless homing mode flag
- Stall detection callback/function
- Different homing sequence for sensorless (no physical endstop reading)

### 5. Integration with Motion Controller

Update `motion_controller.c` to:
- Initialize TMC2209 driver for zoom axis on startup
- Support sensorless homing for zoom axis
- Pass stall detection status to homing state machine

## Implementation Steps

### Step 1: Create TMC2209 UART Driver

```c
// tmc2209.h
bool tmc2209_init(uint8_t axis, uint8_t uart_num);
bool tmc2209_write_register(uint8_t axis, uint8_t address, uint32_t data);
uint32_t tmc2209_read_register(uint8_t axis, uint8_t address);
void tmc2209_set_stallguard_threshold(uint8_t axis, uint8_t threshold);
uint8_t tmc2209_get_stallguard_result(uint8_t axis);
bool tmc2209_is_stalled(uint8_t axis, uint8_t threshold);
```

### Step 2: Configure stallGuard

During initialization:
- Set GCONF.SG_ENABLE = 1
- Set TCOOLTHRS to appropriate value (e.g., 500 steps/sec)
- Set SGTHRS (stallGuard threshold, typically 0-255, start with 150)
- Set motor current appropriately

### Step 3: Update Homing State Machine

Add sensorless homing support:
- Check if axis uses sensorless homing
- Instead of reading GPIO endstop, read TMC2209 SG_RESULT
- Detect stall when SG_RESULT < SGTHRS for consecutive readings
- Use same fast/slow approach + backoff sequence

### Step 4: Calibration

Sensorless homing requires calibration:
1. Set SGTHRS to initial value (e.g., 150)
2. Run homing sequence
3. Adjust SGTHRS based on behavior:
   - Too sensitive (stalls too early): Increase SGTHRS
   - Not sensitive enough (doesn't detect stall): Decrease SGTHRS
4. Repeat until reliable

## TMC2209 UART Protocol

TMC2209 uses a simple UART protocol:
- 8 data bits, 1 stop bit, even parity
- Baud rate: 115200 (configurable)
- Each transaction: 8 bytes
  - Byte 0: Sync byte + slave address (0x05 for read, 0x80 for write)
  - Byte 1: Register address
  - Bytes 2-5: 32-bit data (MSB first)
  - Bytes 6-7: CRC

## Code Structure

```
main/
  ├── tmc2209.h          (New: TMC2209 driver interface)
  ├── tmc2209.c          (New: TMC2209 driver implementation)
  ├── homing.h           (Modify: Add sensorless support)
  ├── homing.c           (Modify: Add stall detection)
  ├── motion_controller.c (Modify: Initialize TMC2209)
  └── board.h            (Modify: Add TMC2209 UART config)
```

## Configuration Parameters

Add to `sdkconfig` or configuration:
- TMC2209 UART baud rate (default: 115200)
- stallGuard threshold (SGTHRS) - per axis
- TCOOLTHRS (minimum velocity for stallGuard)
- Motor current settings (IHOLD, IRUN)

## Testing Sequence

1. **Test UART communication**: Read TMC2209 registers to verify connection
2. **Test stallGuard reading**: Move motor and verify SG_RESULT changes
3. **Calibrate threshold**: Find appropriate SGTHRS value
4. **Test homing**: Run sensorless homing sequence
5. **Verify repeatability**: Run multiple times to ensure consistency

## Limitations

- stallGuard requires motor to be moving (TCOOLTHRS)
- Threshold needs calibration per motor/load combination
- Less reliable than physical endstops in some scenarios
- Requires sufficient motor current for reliable detection

## References

- [TMC2209 Datasheet](https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf)
- [TMC2209 Application Note](https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Application_Note.pdf)
- [FYSETC E4 Board Documentation](https://fysetc.github.io/E4/)

