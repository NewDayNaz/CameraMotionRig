# TMC2209 Current Settings for 17HS4023 (0.7A) Stepper Motor

## Motor Specifications
- **Model**: 17HS4023
- **Rated Current**: 0.7A RMS per phase
- **Recommended Operating Current**: 70-80% of rated = **0.49A - 0.56A RMS**

## Current Setting Approach

Since we're using **UART control** (not VREF potentiometer), current is set via the `IHOLD_IRUN` register:

```
IRUN: 0-31 (controls run current)
IHOLD: 0-31 (controls hold current, typically 50-70% of IRUN)
```

## Formula (UART Mode)

The actual current depends on:
1. **IRUN/IHOLD values** (0-31)
2. **VREF voltage** (set by potentiometer on board)
3. **R_SENSE** (sense resistor, typically 0.11Ω on TMC2209 modules)

For TMC2209 with UART control:
```
I_RMS ≈ (IRUN+1)/32 × I_max
```

Where `I_max` depends on VREF and R_SENSE.

## Recommended Settings

For a **0.7A motor** running at **70-80%** (0.49-0.56A):

### Option 1: Conservative (Currently in code)
```c
IRUN = 12  // ~39% of max
IHOLD = 8  // ~26% of max (67% of IRUN)
```
This is very conservative and should run cool, but may lack torque.

### Option 2: Recommended for 0.7A motor
```c
IRUN = 16-20  // ~50-63% of max
IHOLD = 10-12  // ~31-38% of max (63% of IRUN)
```

### Option 3: Higher torque (if needed)
```c
IRUN = 22-24  // ~69-75% of max
IHOLD = 14-16  // ~44-50% of max (64% of IRUN)
```

## Important Notes

1. **VREF still matters**: Even with UART control, the VREF potentiometer sets the maximum current limit. Make sure VREF is set appropriately (typically 0.9-1.0V for a 0.7A motor).

2. **Start low and increase**: If you're experiencing:
   - **Overheating**: Reduce IRUN
   - **Missed steps/weak motion**: Increase IRUN
   - **Motor not holding position**: Increase IHOLD

3. **Typical IRUN to IHOLD ratio**: 50-70% (IHOLD = 0.5-0.7 × IRUN)

4. **FYSETC E4 Board**: Check your board's documentation for the actual R_SENSE value, as this affects the calculation.

## Current Code Settings

Current code uses:
```c
uint32_t ihold_irun = (8 << 0) | (12 << 8) | (5 << 16);
// IHOLD=8, IRUN=12, IHOLDDELAY=5
```

This is **very conservative** and may need adjustment based on:
- Your specific motor's performance requirements
- Load on the zoom mechanism
- Temperature during operation

## Recommendation

Start with **IRUN=16, IHOLD=10** (Option 2) and adjust based on performance:
- If motor is cool but weak: Increase to IRUN=20, IHOLD=12
- If motor is hot: Reduce to IRUN=12-14, IHOLD=8

Monitor motor temperature during testing!

