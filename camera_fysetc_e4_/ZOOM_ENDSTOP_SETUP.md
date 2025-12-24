# Zoom Axis Endstop Setup

## Configuration

The zoom axis now uses a **physical endstop by default** (GPIO15 / PIN_Z_MIN).

To switch to sensorless homing later, change the configuration in `main/homing.h`:

```c
#define ZOOM_USE_SENSORLESS_HOMING 0  // 0 = physical endstop, 1 = sensorless
```

## Hardware Setup

### Physical Endstop (Default)

1. **Connect endstop to GPIO15** (see `GPIO15_ENDSTOP_WIRING.md` for detailed diagram):
   - **10kΩ resistor** → Connect between **3.3V** and **GPIO15** (REQUIRED for boot)
   - Endstop **COM** → **GPIO15** (same point as resistor)
   - Endstop **NO** → **GND** (when pressed, pulls GPIO15 LOW)
   - If using NC switch: COM → GPIO15, NC → GND

2. **Important**: GPIO15 is a **strap pin** that **MUST be HIGH during boot**
   - **External 10kΩ pullup resistor is REQUIRED** (not optional)
   - Without pullup, ESP32 may not boot correctly
   - Active LOW (endstop triggers when GPIO15 goes LOW)

3. **Boot consideration**: GPIO15 must be HIGH during boot for normal operation
   - With pullup, this should be fine
   - If endstop is pressed during boot, ESP32 may enter download mode
   - Release endstop before powering on if this becomes an issue

### Sensorless Homing (Optional)

To enable sensorless homing:

1. **Set configuration**:
   ```c
   #define ZOOM_USE_SENSORLESS_HOMING 1
   ```

2. **Configure TMC2209 for UART mode**:
   - Check FYSETC E4 board documentation
   - Set jumpers/configure PDN_UART pin
   - See `UART_MODE_REQUIRED.md` for details

3. **No endstop needed** - TMC2209 stallGuard detects stall

## Current Behavior

- **Default**: Zoom uses physical endstop (GPIO15)
- `HOME` command homes all 3 axes: PAN → TILT → ZOOM
- Zoom homing uses same fast/slow approach sequence as PAN/TILT
- Endstop must be at the minimum (home) position of zoom travel

## Testing

1. **Test endstop connection**:
   - Move zoom manually to trigger endstop
   - Check that endstop signal goes LOW when triggered

2. **Test homing**:
   ```
   HOME
   ```
   Should home PAN, then TILT, then ZOOM sequentially

3. **Verify zoom position**:
   ```
   POS
   ```
   ZOOM should be at 0.0 after homing

## Troubleshooting

### Zoom homing doesn't work
- Check endstop wiring (signal, GND, power)
- Verify endstop is at home position
- Check if GPIO15 is being used correctly
- Verify endstop is active LOW

### Boot issues
- GPIO15 pulled LOW during boot can cause problems
- Ensure endstop isn't triggered at power-on
- Consider adding a boot delay in firmware if needed

### Want to switch to sensorless
- Set `ZOOM_USE_SENSORLESS_HOMING = 1` in `homing.h`
- Configure TMC2209 for UART mode
- Rebuild and flash firmware

