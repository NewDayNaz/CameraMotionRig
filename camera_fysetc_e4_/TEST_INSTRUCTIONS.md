# Motor and Endstop Test Program

This test program validates that all motors and endstops are working correctly on the FYSETC E4 board.

## How to Use

1. **Backup your main.c**:
   ```bash
   cd camera_fysetc_e4_
   cp main/main.c main/main.c.backup
   ```

2. **Replace main.c with test program**:
   ```bash
   cp main/test_main.c main/main.c
   ```

3. **Build and flash**:
   ```bash
   idf.py build flash monitor
   ```

4. **Watch the serial output** - the test will:
   - Test each motor (PAN, TILT, ZOOM) by moving them in both directions
   - Test each endstop by reading its state
   - Prompt you to manually trigger endstops to verify they respond

5. **Restore original main.c when done**:
   ```bash
   cp main/main.c.backup main/main.c
   ```

## Test Procedure

The test program will automatically:

1. **Motor Tests** (automatic):
   - Move PAN motor 400 steps in positive direction, then negative
   - Move TILT motor 400 steps in positive direction, then negative
   - Move ZOOM motor 400 steps in positive direction, then negative
   - Watch the motors - they should move smoothly without skipping

2. **Endstop Tests** (manual intervention required):
   - For each endstop:
     - Read the endstop state multiple times
     - Prompt you to manually trigger the endstop
     - Read the state again to verify it changed

3. **Continuous Monitoring**:
   - After tests complete, enters continuous monitoring mode
   - Displays endstop states every second
   - Press Ctrl+C to stop

## Expected Results

- **Motors**: Should move smoothly in both directions without skipping steps
- **Endstops**: 
  - Should read `NOT TRIGGERED` when open (GPIO level = 1)
  - Should read `TRIGGERED` when pressed (GPIO level = 0, active LOW)
  - State should change when you manually trigger them

## Troubleshooting

If a motor doesn't move:
- Check wiring (step, dir, enable, power, ground)
- Verify motor is enabled (enable pin should be LOW for TMC2209)
- Check motor power supply voltage and current
- Verify GPIO pin assignments in `board.h`

If an endstop doesn't respond:
- Check wiring (signal, ground)
- For GPIO34/35: Verify external pullup resistor is installed (10kΩ recommended)
- For GPIO15: Verify external pullup resistor is installed (required for boot)
- Check that endstop switch is working (test with multimeter)
- Verify endstop is active LOW (shorts to ground when triggered)
