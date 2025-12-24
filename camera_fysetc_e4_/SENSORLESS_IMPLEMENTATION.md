# Sensorless Homing Implementation Checklist

This checklist outlines the steps to implement sensorless homing for the zoom axis.

## Prerequisites

- [ ] Understand TMC2209 UART protocol
- [ ] Have TMC2209 datasheet available
- [ ] Verify UART pins are correctly wired on FYSETC E4 board

## Implementation Tasks

### Phase 1: TMC2209 UART Driver (Foundation)

- [ ] **Create `tmc2209.h` and `tmc2209.c`**
  - [ ] Implement UART initialization
  - [ ] Implement write register function (with CRC calculation)
  - [ ] Implement read register function (with CRC verification)
  - [ ] Test basic register read/write

- [ ] **Add to CMakeLists.txt**
  ```cmake
  "tmc2209.c"
  ```

### Phase 2: TMC2209 Configuration

- [ ] **Implement driver initialization**
  - [ ] Set GCONF register (enable UART, enable stallGuard)
  - [ ] Configure motor current (IHOLD_IRUN)
  - [ ] Set TCOOLTHRS (minimum velocity for stallGuard)
  - [ ] Set SGTHRS (stallGuard threshold - start with default)

- [ ] **Add configuration to board.h/c**
  - [ ] Define TMC2209 UART pins
  - [ ] Initialize UART in board_init()

### Phase 3: Stall Detection

- [ ] **Implement stallGuard reading**
  - [ ] Function to read SG_RESULT register
  - [ ] Function to check if motor is stalled
  - [ ] Add filtering/smoothing for reliability

- [ ] **Test stallGuard reading**
  - [ ] Read SG_RESULT while motor is free-running
  - [ ] Read SG_RESULT while motor is blocked/stalled
  - [ ] Verify values change appropriately

### Phase 4: Integrate with Homing System

- [ ] **Update homing.h**
  - [ ] Add flag to indicate sensorless vs physical endstop
  - [ ] Add function to detect stall (for sensorless)

- [ ] **Update homing.c**
  - [ ] Modify homing_update() to check for sensorless mode
  - [ ] Use tmc2209_is_stalled() instead of GPIO read for zoom axis
  - [ ] Ensure same fast/slow/backoff sequence works with stall detection

- [ ] **Update motion_controller.c**
  - [ ] Initialize TMC2209 for zoom axis on startup
  - [ ] Update homing to support zoom axis (sensorless)

### Phase 5: Calibration & Testing

- [ ] **Calibrate stallGuard threshold**
  - [ ] Start with default SGTHRS (150)
  - [ ] Run homing sequence
  - [ ] Adjust SGTHRS based on behavior
  - [ ] Document optimal value for your setup

- [ ] **Test homing sequence**
  - [ ] Verify fast approach detects stall correctly
  - [ ] Verify backoff works
  - [ ] Verify slow approach detects stall correctly
  - [ ] Test repeatability (run multiple times)

- [ ] **Error handling**
  - [ ] Handle UART communication errors
  - [ ] Handle timeout if stall not detected
  - [ ] Add appropriate logging

## Configuration Parameters to Add

Add these to a configuration header or make them configurable:

```c
// In tmc2209.h or config.h
#define ZOOM_STALLGUARD_THRESHOLD  150  // Calibrate this!
#define ZOOM_TCOOLTHRS             500  // Minimum velocity for stallGuard
#define ZOOM_HOLD_CURRENT          16   // Current during standstill (0-31)
#define ZOOM_RUN_CURRENT           20   // Current during operation (0-31)
```

## Testing Procedure

1. **Basic UART Test**
   ```c
   // Read a known register (e.g., GSTAT)
   uint32_t gstat;
   tmc2209_read_register(AXIS_ZOOM, TMC2209_GSTAT, &gstat);
   // Should read 0x00000001 if reset occurred, or 0x00000000
   ```

2. **StallGuard Reading Test**
   ```c
   // Read SG_RESULT while motor moves freely
   uint8_t sg_free = tmc2209_get_stallguard_result(AXIS_ZOOM);
   
   // Block motor manually, read again
   uint8_t sg_stalled = tmc2209_get_stallguard_result(AXIS_ZOOM);
   
   // sg_stalled should be significantly lower than sg_free
   ```

3. **Homing Test**
   ```c
   // Start homing
   motion_controller_home(AXIS_ZOOM);
   
   // Monitor logs to see stall detection
   // Adjust SGTHRS if needed
   ```

## Troubleshooting

**Problem: UART communication fails**
- Check wiring (TX->RX, RX->TX)
- Verify baud rate matches driver configuration
- Check if driver is in UART mode (not standalone mode)

**Problem: stallGuard doesn't detect stall**
- Increase motor current (may need more torque)
- Lower SGTHRS threshold (make it more sensitive)
- Ensure TCOOLTHRS is set correctly (motor must be moving)

**Problem: stallGuard triggers too early**
- Increase SGTHRS threshold (make it less sensitive)
- Check if motor has mechanical resistance

**Problem: Inconsistent stall detection**
- Add filtering (average multiple readings)
- Ensure motor current is stable
- Check for electrical noise

## Notes

- stallGuard works best with adequate motor current
- The threshold (SGTHRS) needs calibration for each motor/load combination
- Sensorless homing is generally less precise than physical endstops but acceptable for zoom axis
- Consider adding a manual calibration command to tune SGTHRS at runtime

