# Motor Overheating When Holding - Fix Applied

## Problem
Motor gets extremely hot when just holding position (idle, not moving).

## Root Cause
**IHOLD current was too high**. Even though IRUN (run current) might be appropriate for motion, IHOLD (hold current) controls the current when the motor is stationary. IHOLD=8 (~26% of max) was still too high for this motor.

## Solution Applied

### 1. Significantly Reduced IHOLD
Changed from:
```c
IHOLD = 8  (~26% of max)
```

To:
```c
IHOLD = 3  (~10% of max)
```

This dramatically reduces power dissipation and heat when the motor is stationary.

### 2. Current Settings
- **IHOLD = 3** (~10% max) - Minimal current for holding position
- **IRUN = 12** (~39% max) - Current during motion (unchanged, already conservative)
- **IHOLDDELAY = 5** - Delay before switching to hold current

## Trade-offs

**Pros:**
- Motor will run much cooler when stationary
- Lower power consumption
- Reduced risk of overheating

**Cons:**
- Slightly less holding torque (may not hold against heavy loads)
- If motor slips when idle, may need to increase IHOLD slightly (to 4-5)

## Testing

After flashing the firmware:

1. **Check idle temperature**: Motor should be cool when not moving
2. **Test holding**: Verify motor still holds position (no slipping)
3. **Test motion**: Verify motor still has enough torque during movement

## If Motor Slipping When Idle

If the motor loses position when idle (slipping), gradually increase IHOLD:
- Try IHOLD=4 first
- Then IHOLD=5 if needed
- Monitor temperature carefully

## If Still Too Hot

If motor still gets hot even with IHOLD=3:
1. Check VREF potentiometer setting on board (may be set too high)
2. Verify motor wiring is correct
3. Check if there's mechanical binding preventing the motor from holding freely
4. Consider if motor specifications match what's needed

## Additional Notes

The TMC2209 has automatic current reduction features (stealthChop bit 10 is enabled), which also helps reduce power consumption during operation. The primary solution is keeping IHOLD as low as possible while still maintaining position.


