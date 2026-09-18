# FYSETC E4 PTZ Camera Rig Firmware

ESP-IDF firmware for a 3-axis PTZ (pan / tilt / zoom) rig on the FYSETC E4 (ESP32 + TMC2209). **Repeatability of presets is the design priority.** Shot composition is open-loop step counting from a trusted origin — the firmware will not invent a home position or snap the step counter to a target.

## Hardware

- **Controller**: FYSETC E4
- **MCU**: ESP32
- **Drivers**: onboard TMC2209 (step/dir, configured over UART1 at boot)
- **Axes**:
  - PAN — X socket: GPIO27 step, GPIO26 dir
  - TILT — Y socket: GPIO33 step, GPIO32 dir
  - ZOOM — Z socket: GPIO14 step, GPIO12 dir
- **Enable**: GPIO25 shared, active LOW
- **PAN / TILT origin**: magnetic inductive sensors on arm mounts, magnets glued on the moving axis. Wired active-LOW into GPIO34 (pan) and GPIO35 (tilt). Those pins are input-only — **external 10 kΩ pullups to 3.3 V required**.
- **ZOOM origin**: no endstop. The lens has a physical stop. Homing uses TMC2209 stallGuard for **one** contact, then pull-off. GPIO15 is **not** used (a floating pin here used to look like a switch and grind the lens 20–30 times).
- **TMC UART**: GPIO22 TX, GPIO21 RX. Addresses: PAN=1, TILT=3, ZOOM=0

There are no encoders. Position is counted STEP pulses.

## What this firmware does (and does not)

It **does**:

- 1 ms FreeRTOS step loop (`vTaskDelayUntil`) with slew-limited jog
- PAN/TILT magnetic homing: backoff if already in the field, fast seek, debounce, pull-off out of the lobe, slow re-approach to the leading edge, final pull-off, `position = 0`
- ZOOM sensorless homing: seek toward the lens stop, stallGuard (or a single range crawl if UART/SG never trips), **one** pull-off, never retry into the glass
- Zoom stallGuard during jog/preset: same threshold as homing; trip faults zoom and clears `homed`
- Idle 6 h with no steps: HOME (no invented zero) then return to the held pose
- Fault (do **not** zero) if a magnet is never seen within `MAX_*_RANGE_STEPS`
- Constant-velocity preset recall with uni-directional backlash take-up
- NVS presets 1–15 (preset 0 is virtual home at 0,0,0)
- USB serial from the Raspberry Pi gamepad service
- HTTP / web UI / MIDI `POST /api/preset/goto`
- TMC2209 UART init: 8 microsteps, spreadCycle, run current; stallGuard enabled on zoom only

It **does not** implement quintic/GPTimer cinematic planning. Smoothness is secondary to hitting the same pose.

## Coordinate frame

| Axis | Home method | Homing direction | Useful travel after home |
|------|-------------|------------------|--------------------------|
| PAN  | magnet leading edge, pulled off | software + | negative steps |
| TILT | magnet leading edge, pulled off | software + | negative steps |
| ZOOM | stall / hard stop, pulled off | software − | positive steps |

SAVE/GOTO are refused until `HOMED=1`. Jogging pan/tilt back into a magnet stops motion, sets a fault, and clears `homed`. Zoom has no switch: stallGuard during jog/preset is the crash stop (same `SG_RESULT` threshold as homing). A trip faults zoom, halts all axes, and clears `homed`. Slow zooms under 25 step/s are not stall-checked (TSTEP saturates).

After **6 hours with no steps**, the rig HOME-s again (does not invent zero) and returns to the pose it was holding. Manual HOME does not restore pose. `STOP` during that home aborts and leaves origin untrusted.

**After flashing this firmware, HOME all axes and re-save every preset.** Pull-off offset changes the origin.

## Homing

**PAN / TILT (magnets)**

1. If the sensor is already LOW (magnet in the lobe), back off until it opens plus extra steps
2. Fast seek toward the magnet
3. Require 12 consecutive 1 ms LOW samples (inductive sensors chatter at the field edge)
4. Pull off until HIGH plus extra steps (leave the detection lobe)
5. Slow seek until debounced LOW (leading edge)
6. Final pull-off so the sensor is not held; set `position = 0`

**ZOOM (no switch)**

1. Seek toward `HOMING_ZOOM_DIRECTION` at 50 step/s
2. Ignore the first 80 steps (startup current spike)
3. Poll stallGuard every 25 ms. Three consecutive `SG_RESULT <= 20` readings = contact
4. Stop and pull off 40 steps so the lens is not jammed on the stop
5. If stallGuard never trips within `MAX_ZOOM_RANGE_STEPS`, treat that far stop as one contact and pull off anyway — **do not seek again**

If a pan/tilt magnet is not found within range, that axis **faults**. Boot will not recall preset 1. `STOP` during homing aborts and leaves origin untrusted.

### Zoom stallGuard tuning

If zoom homes **early** (mid-travel): lower `HOME_ZOOM_SG_STALL_MAX` in [`main/stepper_limits.h`](main/stepper_limits.h) (e.g. 10) and/or lower `TMC_SGTHRS_ZOOM` in [`main/tmc_driver.c`](main/tmc_driver.c).

If zoom **never** stalls and always crawls to range: raise `HOME_ZOOM_SG_STALL_MAX` (e.g. 40) and/or raise `TMC_SGTHRS_ZOOM`. Watch the log line `ZOOM stallGuard SG=...`.

### Live zoom stall-stop

Same SG threshold while jogging or recalling a preset (`|vel| ≥ 25 step/s`, ignore first 40 steps and direction changes). Three low `SG_RESULT` polls halt every axis, fault zoom, and clear `homed`. HOME required after that — the lens stop is no longer a silent grind.

### Idle re-home

After `IDLE_REHOME_MS` (6 hours) with no step pulses, the rig runs a full HOME (still faults if a magnet/stall is missed) and then returns to the step counts it had before that home. Boot HOME still recalls preset 1; this idle pass does not. Manual HOME from the UI does not restore pose.

Calibrate `MAX_*_RANGE_STEPS` by homing, jogging to the far stop, and reading `STATUS`.

## Presets

Stored in NVS as software step counts plus an optional pan/tilt `max_speed`. Accel/decel fields are legacy and ignored. Zoom **never** uses a stored pan `max_speed` (that was skipping the zoom axis).

Recall is constant velocity with a short overshoot so the last motion is always in the software-positive direction. Overshoot is clamped to soft limits. Arrival is counted pulses only — the counter is not written to the target.

SAVE is refused while moving or if not homed.

## USB serial (115200)

- `VEL <pan> <tilt> <zoom>` — jog velocities in steps/sec
- `j,<yaw>,<pitch>,<zoom>` — joystick counts (−32768..32768), scaled to axis max
- `GOTO <n>` / `SAVE <n>` — presets 1–15 (`GOTO 0` is origin)
- `HOME` / `STOP`
- `POS` — `POS:pan,tilt,zoom`
- `STATUS` — `STATUS:PAN:... TILT:... ZOOM:... HOMED:0|1 MOVING:0|1 HOMING:0|1 FAULT:ptz`

Joystick port detection still matches `STATUS:PAN:` … `TILT:` … `ZOOM:`.

## HTTP

- `GET /api/positions` — positions, `homed`, `homing`, `moving`, `endstops`, `faults`, `idle_rehome_s`, `error`
- `POST /api/velocity`, `/api/command` (`home`/`stop`)
- `POST /api/preset/goto` and `/save` — JSON `error` string if not homed / moving
- Web UI at `/` shows homed state; SAVE/GOTO are blocked until HOME succeeds. Zoom endstop will read OPEN (there is none).
- `GET /api/tmc/diag` — UART probe (IC version, IFCNT write-ack, currents, SG)
- `GET /api/tmc/sg?axis=0|1|2` — live stallGuard + TSTEP
- `POST /api/tmc/sg-sample` — jog one axis briefly and record SG
- `POST /api/tmc/reconfigure` — re-push TMC registers

## Heat / hold current

Drivers stay enabled 24/7. Old IHOLD=8 at standstill is enough to cook small steppers after a few days.

Now:
- After ~0.4 s without steps, TMC drops to IHOLD (pan 3, tilt 4, zoom 0)
- After 5 minutes idle, standby IHOLD (pan 1, tilt 2, zoom 0)
- Motion restores IRUN immediately

If **tilt sags** on a long hold, raise `TMC_IHOLD_TILT` in [`main/stepper_limits.h`](main/stepper_limits.h). Probe UART in the web UI and check the reported IHOLD/IRUN.

## Motion notes

- Update task period is 1 ms, so practical pan/tilt ceiling is ~1000 step/s
- Preset defaults: pan/tilt 150 step/s, zoom 40 step/s
- Jog is slew-limited; preset and homing use immediate target velocity
- Recursive mutex serializes HTTP, UART, and the step loop
- Watchdog is fed from the update task

## Building and flashing

See [QUICK_START.md](QUICK_START.md) and [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md).

```cmd
build_and_flash.bat COM3
```

```bash
./build_and_flash.sh /dev/ttyUSB0
```

Copy `main/wifi_config.h.example` to `main/wifi_config.h` and set credentials.

## Pin map

See [`main/board.h`](main/board.h). Enable is GPIO25. Zoom is the **Z** socket (GPIO14/12), not the E0 socket.

## License

See LICENSE.
