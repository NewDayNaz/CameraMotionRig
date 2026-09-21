"""
Vision tuner — Windows NDI calibration tool for the Camera Motion Rig.

Measures image motion from the camera's NDI feed while commanding the ESP32
over HTTP. Optical flow (not a trained model) scores smoothness, lost steps,
travel range, and GOTO ramps. It prints suggested stepper_limits.h values;
it does not flash firmware.

Setup
-----
1. Python 3.10+ on the Windows PC that already sees the NDI source.
2. NDI Runtime 6 is enough (this machine has it at
   `C:\Program Files\NDI\NDI 6 Runtime\v6\`). You do not need the full SDK.
   The tuner puts that folder on the DLL search path before loading cyndilib.
   Override with env `NDI_RUNTIME_DIR` if it is installed somewhere else.
3. pip install -r requirements.txt
   (includes cyndilib). `run.bat` does this.
4. run.bat
   or:  python -m vision_tuner

If the camera encoder is not on the LAN yet, choose Video = demo (synthetic
checkerboard) or webcam to learn the UI. Find NDI again once the source is live.

Use
---
1. Controller URL = the ESP32 web UI (http://x.x.x.x). Connect.
2. Find NDI, pick the program camera, Start video. You need scene texture
   (pews, pulpit, windows) — a black field makes flow look like noise.
3. Stay near the rig. STOP is red. Abort trial cancels a sweep.
   Pan/tilt trials stay within 700/900 steps of the start pose and return
   after each speed so cabling cannot wrap. The full suite does not range-crawl pan/tilt.
4. Start with Stillness, then Sweep pan / tilt / zoom. Read Speed sweep:
   - pan/tilt px/s is image *slide*; zoom px/s is *scale* (FOV in/out)
   - px/s should rise with commanded speed
   - jitter ratio climbing = rough / resonance
   - pixels/step dropping = lost steps
5. Range crawl drives toward travel until the image stops or a fault/PWM hit.
   HOME first. Zoom keeps the rubber-ring PWM stop as a backstop.
6. GOTO duration briefly enables preset automations, recalls the first valid
   preset, then mutes them again.
7. Full auto suite runs sweeps, zoom crawl, GOTO, pan IRUN, and tele pan scale.
8. Suggestions tab compares measurements to current firmware constants.
   Export run writes CSV + JSON + suggestions.txt under tools/vision_tuner/runs/.

NDI can score max speed (done), pan IRUN (lost steps vs current), and
ZOOM_PT_SCALE_MIN (match on-screen pan at tele vs wide). It cannot tune
homing magnets/PWM, zoom IRUN at the rubber ring, or GOTO ease (OBS delay).

Safety
------
Trials mute preset automations. Zoom crawl stops on pwm_hit. Any FAULT stops
motors. Closing the window sends STOP. Do not leave a range crawl unattended.
"""
