# Wiring Regular Endstop Switch to GPIO15 (PIN_Z_MIN)

## Important: GPIO15 Strap Pin Requirement

GPIO15 is a **strap pin** on ESP32 that **must be HIGH during boot**. This requires an **external pullup resistor**.

## Simple Wiring (Recommended)

### Parts Needed:
- Mechanical endstop switch (NO - Normally Open, or NC - Normally Closed)
- **10kΩ resistor** (1/4W or 1/8W) - **REQUIRED for boot**
- Jumper wires

### Wiring Diagram:

```
For NO (Normally Open) Switch:
┌─────────────────┐
│   Endstop       │    10kΩ Resistor
│   Switch        │    (Pullup)
│                 │    │
│  [COM] ─────────┼────┼── GPIO15 (ESP32)
│                 │    │
│  [NO]           │    │
│                 │    │
└─────────────────┘    │
                       │
                       │
                     3.3V (ESP32)

When NOT pressed: GPIO15 = HIGH (via pullup)
When pressed: COM connects to NO, GPIO15 = LOW (triggered)
```

### Step-by-Step:

1. **Connect 10kΩ resistor**:
   - One end → **3.3V** (ESP32 power)
   - Other end → **GPIO15**

2. **Connect endstop switch**:
   - **COM terminal** → **GPIO15** (same point as resistor)
   - **NO terminal** → **GND** (ESP32 ground)
   - If your switch is **NC (Normally Closed)**, reverse: COM → GPIO15, NC → GND

3. **Operation**:
   - **Not pressed**: Pullup resistor pulls GPIO15 HIGH (normal state)
   - **Pressed**: Switch connects GPIO15 to GND, goes LOW (trigger detected)
   - **During boot**: Pullup ensures GPIO15 is HIGH → normal boot

## Alternative: Using Board's 3-Pin Connector

If your board has a 3-pin endstop connector (VCC, GND, SIG):

**Option A: With External Pullup (Recommended for GPIO15)**
```
Board Connector          Endstop Switch
┌─────────────┐         ┌──────────────┐
│  VCC (3.3V) │─────────┤  10kΩ Res ───┼─ GPIO15/SIG
│             │         │              │
│  GND        │─────────┤  GND         │
│             │         │              │
│  SIG        │─────────┤  COM ────────┼─ GPIO15
│  (GPIO15)   │         │              │
└─────────────┘         │  NO          │
                        └──────────────┘
                         NO → GND (when pressed, pulls SIG LOW)
```

**Option B: If Connector Has Built-in Pullup**
- Check board documentation
- If pullup exists, you may not need external resistor
- But GPIO15 may still need stronger pullup for reliable boot

## Boot Safety

**Critical:** GPIO15 must be HIGH during boot!

**Safe procedure:**
1. Power OFF ESP32
2. Ensure endstop is **NOT pressed** (GPIO15 = HIGH via pullup)
3. Power ON ESP32
4. Wait for boot
5. Endstop can now be used normally

**If endstop pressed during boot:**
- ESP32 may enter download mode
- System won't boot normally
- **Fix:** Release endstop, power cycle

## Testing Your Wiring

1. **Check with multimeter** (power OFF):
   - GPIO15 to 3.3V: Should see 10kΩ (pullup resistor)
   - GPIO15 to GND: Should be open (when switch not pressed)
   - Press switch: GPIO15 to GND should be short (switch closed)

2. **Check with multimeter** (power ON):
   - GPIO15 voltage: Should be ~3.3V when switch NOT pressed
   - Press switch: GPIO15 voltage should drop to ~0V

3. **Test homing**:
   ```
   HOME
   ```
   Should detect endstop when pressed

## Troubleshooting

### Boot Issues
- **Symptom:** ESP32 won't boot or enters download mode
- **Cause:** GPIO15 LOW during boot
- **Fix:** 
  - Add/check 10kΩ pullup resistor
  - Ensure endstop isn't pressed at power-on
  - Try stronger pullup (4.7kΩ) if issues persist

### Endstop Not Detected
- **Symptom:** Homing doesn't trigger when endstop pressed
- **Check:**
  - COM connected to GPIO15?
  - Pullup resistor connected (3.3V → GPIO15)?
  - Switch actually closing when pressed?
  - Measure voltage: Should be LOW when pressed

### False Triggers
- **Symptom:** Endstop triggers when not pressed
- **Check:**
  - Pullup resistor value (10kΩ recommended)
  - Wiring (no shorts to GND)
  - Switch condition (may be faulty or stuck)

## Current Firmware Configuration

The firmware expects:
- **GPIO15** (PIN_Z_MIN) as input
- **Active LOW** (GPIO LOW = endstop triggered)
- **No internal pullup** (external required)

This matches PAN (GPIO34) and TILT (GPIO35) endstops.
