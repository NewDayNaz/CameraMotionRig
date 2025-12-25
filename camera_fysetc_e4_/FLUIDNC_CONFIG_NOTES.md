# FluidNC Configuration Integration Notes

This document describes how the FluidNC configuration has been integrated into the board configuration.

## TMC2209 Driver Addresses

Based on the FluidNC config, the TMC2209 drivers use the following UART addresses (set via hardware PDN_UART pin configuration):

- **PAN (X axis)**: Address 1
- **TILT (Y axis)**: Address 3  
- **ZOOM (Z axis)**: Address 0

All drivers share the same UART bus (UART1) at 115200 baud.

## Pin Mapping (from FluidNC config)

### Motor Pins
- **PAN (X)**:
  - Step: GPIO27
  - Dir: GPIO26
  - Enable: GPIO25 (shared)

- **TILT (Y)**:
  - Step: GPIO33
  - Dir: GPIO32
  - Enable: GPIO25 (shared)

- **ZOOM (Z)**:
  - Step: GPIO14
  - Dir: GPIO12
  - Enable: GPIO25 (shared)

### Endstop Pins
- **PAN (X)**: GPIO15 (limit_neg_pin)
  - ⚠️ **WARNING**: GPIO15 is a boot strap pin. Must have external pullup (10kΩ to 3.3V) for proper boot behavior.

- **TILT (Y)**: GPIO35 (motor0 limit_neg_pin)
  - ⚠️ **NOTE**: GPIO35 is input-only, requires external pullup (10kΩ to 3.3V)

- **ZOOM (Z)**: GPIO34 (Z axis limit_neg_pin)
  - ⚠️ **NOTE**: GPIO34 is input-only, requires external pullup (10kΩ to 3.3V)

### UART Pins (TMC2209 communication)
- TX: GPIO22
- RX: GPIO21
- Baud: 115200
- Parity: Even (required by TMC2209)
- Mode: 8N1

## Important Notes

1. **TMC2209 UART Addressing**: The driver addresses are set via hardware (PDN_UART pin configuration on the TMC2209 chips), not via software. The addresses in the code are for reference and logging only.

2. **Enable Pin**: GPIO25 is shared by all drivers and is active LOW (TMC2209 standard). When set to LOW, drivers are enabled.

3. **Endstop Pullups**: All endstop pins require external pullup resistors:
   - GPIO15: **REQUIRED** for proper boot (strap pin)
   - GPIO34/35: Required for proper signal levels (input-only pins)

4. **Motor and Endstop Mapping**: Each axis now uses its dedicated motor pins and endstop:
   - PAN uses X motor pins (GPIO27/26) and X endstop (GPIO15)
   - TILT uses Y motor pins (GPIO33/32) and Y endstop (GPIO35)
   - ZOOM uses Z motor pins (GPIO14/12) and Z endstop (GPIO34)

## Configuration Changes

The following files have been updated:

- `board.h`: Added TMC2209 address definitions and helper function
- `board.c`: Added address mapping array and getter function, updated endstop pin mapping
- `tmc2209.c`: Added address logging in read/write operations
- `test_main.c`: Added address display in test output

## Testing

The test program (`test_main.c`) now displays:
- TMC2209 driver address for each axis
- UART configuration (TX/RX pins, baud rate)
- Endstop pin mappings

This helps verify that the configuration matches the hardware setup.
