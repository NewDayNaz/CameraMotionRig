# Sensorless Homing Implementation Summary

## What Was Implemented

All features from `SENSORLESS_IMPLEMENTATION.md` have been implemented:

### ✅ Phase 1: TMC2209 UART Driver
- ✅ Created `tmc2209.h` and `tmc2209.c`
- ✅ Implemented UART initialization with even parity
- ✅ Implemented write register function with CRC calculation
- ✅ Implemented read register function with CRC verification
- ✅ Added to CMakeLists.txt

### ✅ Phase 2: TMC2209 Configuration
- ✅ Implemented driver initialization (`tmc2209_init()`)
- ✅ Configure GCONF register (enable UART, enable stallGuard)
- ✅ Configure motor current (IHOLD_IRUN)
- ✅ Set TCOOLTHRS (minimum velocity for stallGuard)
- ✅ Set SGTHRS (stallGuard threshold - defaults to 150)

### ✅ Phase 3: Stall Detection
- ✅ Implemented stallGuard reading (`tmc2209_get_stallguard_result()`)
- ✅ Implemented stall detection function (`tmc2209_is_stalled()`)
- ✅ Added debouncing (requires 3 consecutive stall readings)

### ✅ Phase 4: Integration with Homing System
- ✅ Updated `homing.h`: Added sensorless support flags and functions
- ✅ Updated `homing.c`: Modified homing_update() to support sensorless mode
- ✅ Uses `tmc2209_is_stalled()` for zoom axis instead of GPIO read
- ✅ Fast/slow approach + backoff sequence works with stall detection
- ✅ Updated `motion_controller.c`: Initialize TMC2209 on startup
- ✅ Sequential homing now supports all 3 axes (PAN, TILT, ZOOM)

## Key Features

1. **Automatic Detection**: Zoom axis automatically uses sensorless homing, PAN/TILT use physical endstops

2. **Sequential Homing**: `HOME` command now homes all 3 axes:
   - PAN (physical endstop)
   - TILT (physical endstop)  
   - ZOOM (sensorless/stallGuard)

3. **Stall Detection**: Uses debouncing (3 consecutive readings) for reliable stall detection

4. **Configuration**: Default stallGuard threshold is 150, can be adjusted via `tmc2209_set_stallguard_threshold()`

## Files Modified/Created

### New Files:
- `main/tmc2209.h` - TMC2209 driver interface
- `main/tmc2209.c` - TMC2209 driver implementation

### Modified Files:
- `main/homing.h` - Added sensorless support
- `main/homing.c` - Integrated stall detection
- `main/motion_controller.c` - Initialize TMC2209, support zoom homing
- `main/CMakeLists.txt` - Added tmc2209.c to build
- `README.md` - Updated documentation

## Configuration Parameters

Default values (can be adjusted in `tmc2209.h`):
- `TMC2209_DEFAULT_SGTHRS`: 150 (stallGuard threshold, 0-255)
- `TMC2209_DEFAULT_TCOOLTHRS`: 500 (minimum velocity for stallGuard, steps/sec)
- `STALL_DEBOUNCE_COUNT`: 3 (consecutive stall readings required)

## Calibration Required

⚠️ **Important**: The stallGuard threshold (`SGTHRS`) needs to be calibrated for your specific motor/load:

1. Start with default (150)
2. Run homing sequence
3. Adjust based on behavior:
   - Too sensitive (stalls too early): Increase threshold
   - Not sensitive enough: Decrease threshold
4. Use `tmc2209_set_stallguard_threshold(AXIS_ZOOM, new_value)` to adjust

## Testing

To test sensorless homing:

1. **Verify UART Communication**:
   ```c
   uint32_t gstat;
   tmc2209_read_register(AXIS_ZOOM, TMC2209_GSTAT, &gstat);
   // Should succeed
   ```

2. **Test Stall Detection**:
   ```c
   uint8_t sg_free = tmc2209_get_stallguard_result(AXIS_ZOOM);
   // Block motor manually
   uint8_t sg_stalled = tmc2209_get_stallguard_result(AXIS_ZOOM);
   // sg_stalled should be lower than sg_free
   ```

3. **Test Homing**:
   ```
   HOME
   ```
   Should home all 3 axes sequentially.

## Notes

- TMC2209 UART uses even parity (configured in `tmc2209_uart_init()`)
- All drivers share the same UART bus (may need addressing if using multiple)
- Sensorless homing is less precise than physical endstops but acceptable for zoom axis
- Motor must be moving above TCOOLTHRS for stallGuard to work

## Next Steps

1. Build and flash firmware
2. Calibrate stallGuard threshold for your zoom motor
3. Test homing sequence multiple times for reliability
4. Adjust threshold as needed

