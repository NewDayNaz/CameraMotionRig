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
- **ZOOM origin**: no endstop. The lens has a rubber-ring stop at wide and tele. Homing seeks **wide** until stealthChop `PWM_SCALE_SUM` rises (load at the ring), sets that contact as 0, then pulls off 32 steps. GPIO15 is **not** used. StallGuard is logged during calibration but does **not** home or crash-stop zoom — skip on the pinion looks like free-run on SG.
- **TMC UART**: GPIO22 TX, GPIO21 RX. Addresses: PAN=1, TILT=3, ZOOM=0. The E4 only ties those pins to the TMC PDN bus when two shunts sit on the I2C/USART1 column (P17) to the center TMC column (P18).

There are no encoders. Position is counted STEP pulses.

## What this firmware does (and does not)

It **does**:

- 1 ms FreeRTOS step loop (`vTaskDelayUntil`) with slew-limited jog
- PAN/TILT magnetic homing: backoff if already in the field, fast seek, debounce, pull-off out of the lobe, slow re-approach to the leading edge, final pull-off, `position = 0`
- ZOOM PWM homing: if PWM is already high, peek toward tele; seek wide at the calibration speed (80 step/s); trip when `PWM_SCALE_SUM` stays above the calibrated threshold (default 81); pull off 32 steps. UART-down falls back to a short step crawl
- Zoom calibration (web wizard): free-run + auto-mark wide/tele on that PWM rise; save stores span and soft limits 32 steps inside each rubber end. Live stallGuard stays **off** unless free-run SG is 20+ above both ends (it is not, on this lens)
- Jog/preset slow in the last 40 steps before a software limit
- Idle 6 h with no steps: HOME (no invented zero) then return to the held pose
- Fault (do **not** zero) if a magnet is never seen within `MAX_*_RANGE_STEPS`, or if zoom PWM never rises
- Duration-based preset recall with ease in/out and uni-directional backlash take-up
- NVS presets 1–15 with optional names (preset 0 is virtual home at 0,0,0)
- USB serial from the Raspberry Pi gamepad service
- HTTP / web UI / MIDI `POST /api/preset/goto`
- TMC2209 UART init: 8 microsteps; pan/tilt spreadCycle; zoom stealthChop; IRUN from NVS (defaults pan/tilt CS 11, zoom CS 5)

It **does not** implement quintic/GPTimer cinematic planning. GOTO eases with a trapezoid; destination is still counted pulses.

## Coordinate frame

| Axis | Home method | Homing direction | Useful travel after home |
|------|-------------|------------------|--------------------------|
| PAN  | magnet leading edge, pulled off | software + | negative steps |
| TILT | magnet leading edge, pulled off | software + | negative steps |
| ZOOM | PWM_SCALE_SUM at wide rubber, pulled off 32 | software − | positive steps (0 = rubber) |

SAVE/GOTO are refused until `HOMED=1`. Jogging pan/tilt back into a magnet stops motion, sets a fault, and clears `homed`. Zoom has no switch: **soft limits from calibration** keep jog short of each rubber end. Live stallGuard is off on this mechanics (SG bands overlap). After a reboot, HOME must PWM-seek wide again — the step counter is 0 even if the lens is mid-travel.

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

**ZOOM (no switch — PWM at the rubber)**

Both lens ends raise `PWM_SCALE_SUM` (~84–86) vs free-run (~76–78) at 80 step/s. StallGuard does not separate skip from air.

1. If PWM is already high, peek 12 steps toward tele (leave wide, or reverse off tele)
2. Seek toward `HOMING_ZOOM_DIRECTION` (wide) at `HOME_ZOOM_CAL_SAMPLE_VEL` (80 step/s)
3. Ignore a short startup window, then require three polls with PWM ≥ calibrated trip (default 81, midpoint of free vs ends after Save)
4. Set that contact as position 0, pull off `HOME_ZOOM_CAL_MARGIN` (32) toward tele
5. If UART is down, fall back to `HOME_ZOOM_DRIVE_STEPS` and invent zero — do not use this path if UART works
6. If PWM never rises within the seek cap, **fault** zoom (do not invent origin)

If a pan/tilt magnet is not found within range, that axis **faults**. The same for zoom if PWM never rises. A fault on any axis aborts HOME and clears `homed`. Boot HOME then recalls preset 1 if stored. `STOP` during homing aborts and leaves origin untrusted.

Pan, tilt, and zoom home **at the same time**. Zoom seek ignores only a short startup window so a HOME from the pulled-off wide pose does not drive through the rubber before looking for PWM.

### Zoom calibration (web UI)

**First-run setup** wizard: HOME → pan/tilt/zoom IRUN → free-run + wide + tele → Save. Daily Drive stays on the main page.

1. HOME so pan/tilt magnets are trusted and zoom PWM-homes to wide
2. Set IRUN (zoom default CS 5 so the pinion stalls instead of skipping)
3. Capture free-run (auto-moves, samples SG/PWM)
4. Capture wide and tele — each button drives that way and **auto-marks when PWM rises**; click again to mark by ear
5. Save. Span is rebased so wide = 0. Soft travel is 32..(span−32). Live stall stays off when SG overlaps

After Save, jog stops 32 steps short of each ring and eases speed in the last 40 steps. Status shows zoom as % of the saved span plus IRUN.

### Live zoom stall-stop

Off unless calibration finds free-run `SG_RESULT` min at least 20 above both rubber ends. On this lens it does not. Do not lower the SG gap to force it on — skip still looks like free-run and would false-trip mid-travel.

### Idle re-home

After `IDLE_REHOME_MS` (6 hours) with no step pulses, the rig runs a full HOME (still faults if a magnet/PWM end is missed) and then returns to the step counts it had before that home. Boot HOME still recalls preset 1; this idle pass does not. Manual HOME from the UI does not restore pose.

Calibrate `MAX_*_RANGE_STEPS` by homing, jogging to the far stop, and reading `STATUS`.

## Presets

Stored in NVS as software step counts, an optional short **name**, and a shared **duration** in seconds. Older blobs without name/duration still load. Accel/decel fields are unused. A leftover pan/tilt `max_speed` is only used when duration is 0 (legacy).

Recall times all axes to the same duration (0 = auto from default pan/tilt 127.5 and zoom 45 step/s). Speed is clamped per axis so a long pan cannot force zoom past its max. Moves ease in and out over ~0.4 s at cruise; a short overshoot keeps the last motion software-positive. Arrival is counted pulses only — the counter is not written to the target.

SAVE is refused while moving or if not homed. Saving the current pose keeps the existing name and duration.

**Preset automations** can be turned off from the web **Presets ON/OFF** button, Companion (`POST /api/preset/recall`), MIDI note 16, or serial `AUTO 0`. While off, every GOTO is ignored (including boot recall of preset 1). Jog, STOP, HOME, and SAVE still work. A move already in progress is halted. The flag is stored in NVS so it survives a reboot.

## USB serial (115200)

- `VEL <pan> <tilt> <zoom>` — jog velocities in steps/sec
- `j,<yaw>,<pitch>,<zoom>` — joystick counts (−32768..32768), scaled to axis max
- `GOTO <n>` / `SAVE <n>` — presets 1–15 (`GOTO 0` is origin)
- `HOME` / `STOP`
- `POS` — `POS:pan,tilt,zoom`
- `STATUS` — `STATUS:PAN:... TILT:... ZOOM:... HOMED:0|1 MOVING:0|1 HOMING:0|1 FAULT:ptz AUTO:0|1`
- `AUTO` / `AUTO 0` / `AUTO 1` — toggle or set preset automations

Joystick port detection still matches `STATUS:PAN:` … `TILT:` … `ZOOM:`.

## HTTP

- `GET /api/positions` — positions, `homed`, `homing`, `moving`, `endstops`, `faults`, `idle_rehome_s`, `error`, zoom soft range / `%` of span, `pan_irun` / `tilt_irun` / `zoom_irun`
- `POST /api/velocity`, `/api/command` (`home`/`stop`)
- `POST /api/preset/goto` and `/save` — JSON `error` string if not homed / moving; `name` when the slot has one. GOTO is ignored while preset automations are off.
- `GET`/`POST /api/preset/recall` — `{"enabled": true|false}` or `{"toggle": true}`. Off = ignore MIDI/Companion/web/serial GOTO; jog and SAVE still work. Persisted in NVS.
- `POST /api/command` — `home` / `stop` / `preset_recall_on` / `preset_recall_off` / `preset_recall_toggle`
- `GET /api/presets` — all 16 slots (`index`, `name`, `duration_s`, `valid`, `pos`)
- `GET /api/preset/get?index=` / `POST /api/preset/update` — name, duration, positions
- `GET`/`POST /api/zoom-cal` — capture free/wide/tele, save, clear
- `GET`/`POST /api/tmc/irun` — pan/tilt/zoom CS 3–16, persisted in NVS
- Web UI at `/` — Drive (home, stop, joysticks, presets) on top; Setup wizards at the bottom (first-run, zoom cal, motor current, TMC debug). SAVE/GOTO blocked until HOME succeeds. Zoom endstop reads OPEN (there is none).
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
- Live IRUN is in **Motor calibration** (and the first-run wizard)

If **tilt sags** on a long hold, raise `TMC_IHOLD_TILT` in [`main/stepper_limits.h`](main/stepper_limits.h). Probe UART in the web UI and check the reported IHOLD/IRUN.

## Motion notes

- Update task period is 1 ms, so practical pan/tilt ceiling is ~1000 step/s
- Preset defaults: pan/tilt 127.5 step/s, zoom 45 step/s (used when duration is 0). Jog pan/tilt scale with the **calibrated zoom span**, not `MAX_ZOOM_RANGE_STEPS`. Max jog is pan 722.5 / tilt 1020 step/s.
- Jog is slew-limited; preset and homing use immediate target velocity
- Recursive mutex serializes HTTP, UART, and the step loop
- Watchdog is fed from the update task
- Zoom UART (SG/TSTEP/PWM) is polled on a background task — never from the 1 ms step loop

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
