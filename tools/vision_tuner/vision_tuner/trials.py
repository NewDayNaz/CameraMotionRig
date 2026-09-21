from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

from vision_tuner.analyze import (
    GotoResult,
    IrunPoint,
    RangeResult,
    SweepPoint,
    TunerReport,
    ZoomPtScaleResult,
    suggest_zoom_pt_scale,
)
from vision_tuner.config import (
    AXIS_MAX_RANGE,
    AXIS_MAX_VEL,
    FIRMWARE,
    SWEEP_MEASURE_S,
    SWEEP_SPEEDS,
    SWEEP_SPINUP_S,
    TRAVEL_SIGN,
    TUNER_TRAVEL_BUDGET,
)
from vision_tuner.controller import Controller, RigStatus
from vision_tuner.vision import VisionMetrics


@dataclass
class Sample:
    t: float
    pan: float = 0.0
    tilt: float = 0.0
    zoom: float = 0.0
    cmd_pan: float = 0.0
    cmd_tilt: float = 0.0
    cmd_zoom: float = 0.0
    flow_dx: float = 0.0
    flow_dy: float = 0.0
    flow_mag: float = 0.0
    jitter: float = 0.0
    radial: float = 0.0
    px_per_s: float = 0.0
    zoom_px_s: float = 0.0
    sharpness: float = 0.0
    texture: float = 0.0
    low_texture: bool = False
    still: bool = True
    moving: bool = False
    homing: bool = False
    fault: bool = False
    pwm_hit: bool = False
    trial: str = ""

    def vis_px_s(self, axis: str = "") -> float:
        """Image speed for this axis. Zoom is scale (radial), not a slide."""
        if axis == "zoom":
            return self.zoom_px_s
        if axis in ("pan", "tilt"):
            return self.px_per_s
        return max(self.px_per_s, self.zoom_px_s)


class Recorder:
    def __init__(self, maxlen: int = 20000):
        self.samples: deque[Sample] = deque(maxlen=maxlen)
        self.trial_name = ""

    def clear(self) -> None:
        self.samples.clear()

    def add(self, s: Sample) -> None:
        s.trial = self.trial_name
        self.samples.append(s)

    def since(self, t0: float) -> list[Sample]:
        return [s for s in self.samples if s.t >= t0]


LogFn = Callable[[str], None]
CheckFn = Callable[[], bool]


class TrialAborted(Exception):
    pass


class TrialRunner:
    def __init__(
        self,
        ctl: Controller,
        rec: Recorder,
        latest_metrics: Callable[[], Optional[VisionMetrics]],
        should_abort: CheckFn,
        log: LogFn,
    ):
        self.ctl = ctl
        self.rec = rec
        self.latest_metrics = latest_metrics
        self.should_abort = should_abort
        self.log = log
        self.report = TunerReport()
        self._origin = {"pan": 0.0, "tilt": 0.0, "zoom": 0.0}

    def _abort_if(self) -> None:
        if self.should_abort():
            self.ctl.stop()
            raise TrialAborted("aborted")

    def _status(self, zoom_safety: bool = False) -> RigStatus:
        st = self.ctl.positions()
        if not st.ok:
            raise TrialAborted(st.message or "controller unreachable")
        if zoom_safety:
            st = self.ctl.refresh_zoom_safety(st)
        if st.any_fault:
            self.ctl.stop()
            raise TrialAborted(f"FAULT {st.faults} {st.error}".strip())
        return st

    def _sleep(self, seconds: float, zoom_safety: bool = False, axis: Optional[str] = None) -> None:
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            self._abort_if()
            st = self._status(zoom_safety=zoom_safety)
            if axis and self._over_budget(axis, st):
                self.ctl.stop()
                raise TrialAborted(f"{axis} travel cap {TUNER_TRAVEL_BUDGET[axis]} steps (cables)")
            time.sleep(0.05)

    def _capture_origin(self, st: Optional[RigStatus] = None) -> None:
        st = st or self._status()
        self._origin = {"pan": st.pan, "tilt": st.tilt, "zoom": st.zoom}

    def _over_budget(self, axis: str, st: RigStatus) -> bool:
        used = abs(st.pos(axis) - self._origin[axis])
        return used >= TUNER_TRAVEL_BUDGET[axis]

    def _return_to_origin(self, axis: str) -> None:
        st = self._status()
        origin = self._origin[axis]
        err = st.pos(axis) - origin
        if abs(err) < 40:
            self.ctl.stop()
            return
        speed = 160.0 if axis != "zoom" else 80.0
        sign = 1.0 if err < 0 else -1.0
        self.log(f"{axis} returning toward start ({st.pos(axis):.0f} → {origin:.0f})")
        self._vel(axis, sign * speed)
        timeout = abs(err) / max(speed, 1.0) + 2.0
        t0 = time.perf_counter()
        try:
            while time.perf_counter() - t0 < timeout:
                self._abort_if()
                st = self._status(zoom_safety=(axis == "zoom"))
                now = st.pos(axis) - origin
                if abs(now) < 50:
                    break
                if err < 0 and now >= 0:
                    break
                if err > 0 and now <= 0:
                    break
                if axis == "zoom" and st.pwm_hit:
                    break
                time.sleep(0.05)
        finally:
            self.ctl.stop()
        self._sleep(0.2)

    def _vel(self, axis: str, speed: float) -> None:
        v = {"pan": 0.0, "tilt": 0.0, "zoom": 0.0}
        v[axis] = speed
        self.ctl.set_velocity(**v)

    def _ndi_burst(self, axis: str, speed: float) -> dict:
        """Short move, stop, wait for delayed NDI, score, return that axis to origin."""
        st0 = self._status(zoom_safety=(axis == "zoom"))
        if self._over_budget(axis, st0):
            return {"mean_px_s": 0.0, "mean_mag": 0.0, "mean_jitter": 0.0, "steps": 0.0, "aborted": "travel cap"}
        direction = self._travel_dir(axis, st0)
        budget = TUNER_TRAVEL_BUDGET[axis]
        burst = min(SWEEP_MEASURE_S.get(axis, 0.70), 0.55 * budget / max(speed, 1.0))
        burst = max(0.35, burst)
        self.ctl.stop()
        self._sleep(0.15)
        st0 = self._status(zoom_safety=(axis == "zoom"))
        t_cmd = time.perf_counter()
        try:
            self._vel(axis, direction * speed)
            self._sleep(burst, zoom_safety=(axis == "zoom"), axis=axis)
        except TrialAborted as e:
            self.ctl.stop()
            if "travel cap" in str(e):
                return {"mean_px_s": 0.0, "mean_mag": 0.0, "mean_jitter": 0.0, "steps": 0.0, "aborted": "travel cap"}
            raise
        finally:
            self.ctl.stop()
        st1 = self._status(zoom_safety=(axis == "zoom"))
        steps = abs(st1.pos(axis) - st0.pos(axis))
        t_vis = self.wait_for_image_motion(t_cmd, timeout=6.0, axis=axis)
        if t_vis <= 0:
            self.wait_for_image_still(2.0, axis=axis)
            self._return_to_origin(axis)
            return {
                "mean_px_s": 0.0,
                "mean_mag": 0.0,
                "mean_jitter": 0.0,
                "steps": steps,
                "aborted": "ndi_timeout",
            }
        self._sleep(0.85)
        meas = self._measure_window(t_vis, time.perf_counter(), axis)
        meas["steps"] = steps
        meas["aborted"] = ""
        self.wait_for_image_still(2.5, axis=axis)
        self._return_to_origin(axis)
        return meas

    def _travel_dir(self, axis: str, st: RigStatus) -> float:
        sign = TRAVEL_SIGN[axis]
        if axis == "zoom":
            span = st.zoom_span or AXIS_MAX_RANGE["zoom"]
            if span > 0 and st.zoom > 0.72 * span:
                return -sign
            return sign
        limit = AXIS_MAX_RANGE[axis]
        if abs(st.pos(axis)) > 0.72 * limit:
            return -sign
        return sign

    def wait_still(self, seconds: float = 1.2) -> float:
        self.ctl.stop()
        self._sleep(seconds)
        chunk = self.rec.since(time.perf_counter() - seconds)
        if not chunk:
            self.log("No NDI samples during stillness — is video started?")
            self.report.stillness_floor = 0.0
            self.report.stillness_floor_px_s = 0.02
            self.report.stillness_zoom_px_s = 0.02
            return 0.02
        mags = sorted(s.flow_mag for s in chunk)
        pxs = sorted(s.px_per_s for s in chunk)
        zxs = sorted(s.zoom_px_s for s in chunk)
        floor = mags[len(mags) // 2]
        floor_px_s = pxs[len(pxs) // 2]
        floor_zoom = zxs[len(zxs) // 2] if zxs else 0.02
        self.report.stillness_floor = floor
        self.report.stillness_floor_px_s = floor_px_s
        self.report.stillness_zoom_px_s = floor_zoom
        return floor_px_s

    def stillness_baseline(self) -> float:
        self.rec.trial_name = "still"
        self.log("Measuring stillness (2.0s)…")
        floor_px_s = self.wait_still(2.0)
        self.log(
            f"Stillness floor {self.report.stillness_floor:.4f} px/frame  "
            f"slide {floor_px_s:.2f} px/s  zoom {self.report.stillness_zoom_px_s:.2f} px/s"
        )
        return floor_px_s

    def wait_home(self, timeout: float = 90.0) -> None:
        self.log("HOME…")
        self.ctl.home()
        t0 = time.perf_counter()
        seen_homing = False
        while time.perf_counter() - t0 < timeout:
            self._abort_if()
            st = self._status()
            if st.homing:
                seen_homing = True
            if seen_homing and not st.homing and st.homed:
                self.log("HOME done")
                self.wait_still(1.0)
                return
            if (not st.homing) and st.error:
                raise TrialAborted(st.error)
            time.sleep(0.15)
        raise TrialAborted("HOME timed out")

    def _measure_window(self, t0: float, t1: float, axis: str) -> dict:
        chunk = [s for s in self.rec.since(t0) if s.t <= t1]
        if len(chunk) < 4:
            return {
                "mean_px_s": 0.0,
                "mean_mag": 0.0,
                "mean_jitter": 0.0,
                "mean_radial": 0.0,
                "still_frac": 1.0,
                "low_texture": True,
                "steps": 0.0,
            }
        mid = chunk[len(chunk) // 8 : -max(1, len(chunk) // 10)] or chunk
        vis = [s.vis_px_s(axis) for s in mid]
        if axis == "zoom":
            mag = [abs(s.radial) for s in mid]
            still_cut = max(0.12, 2.5 * max(min(self.report.stillness_zoom_px_s or 0.02, 0.12), 0.02))
        else:
            mag = [s.flow_mag for s in mid]
            still_cut = max(0.12, 3.0 * max(min(self.report.stillness_floor_px_s, 0.08), 0.02))
        jit = [s.jitter for s in mid]
        rad = [s.radial for s in mid]
        still = sum(1 for v in vis if v < still_cut) / float(len(vis))
        low = sum(1 for s in mid if s.low_texture) > len(mid) * 0.5
        steps = abs(getattr(chunk[-1], axis) - getattr(chunk[0], axis))
        mean_px = sum(vis) / len(vis)
        mean_j = sum(jit) / len(jit)
        return {
            "mean_px_s": mean_px,
            "mean_mag": sum(mag) / len(mag),
            "mean_jitter": mean_j,
            "mean_radial": sum(rad) / len(rad),
            "still_frac": still,
            "low_texture": low,
            "steps": steps,
        }

    def _motion_need(self, axis: str = "") -> float:
        if axis == "zoom":
            floor = self.report.stillness_zoom_px_s
            if floor <= 0.03:
                return 0.18
            return max(0.14, 2.5 * min(floor, 0.12))
        floor = self.report.stillness_floor_px_s
        if floor <= 0.03:
            return 0.20
        return max(0.15, 3.0 * min(floor, 0.08))

    def wait_for_image_motion(self, t_after: float, timeout: float = 6.0, axis: str = "") -> float:
        """First sample time after t_after where NDI shows motion. 0 if none."""
        need = self._motion_need(axis)
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            self._abort_if()
            self._status()
            for s in self.rec.since(t_after):
                if s.vis_px_s(axis) >= need:
                    lat = s.t - t_after
                    if lat >= 0:
                        self.report.latency_s = lat
                    return s.t
            time.sleep(0.05)
        return 0.0

    def wait_for_image_still(self, timeout: float = 4.0, axis: str = "") -> None:
        need = self._motion_need(axis)
        hits = 0
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            self._abort_if()
            m = self.latest_metrics()
            if m is None:
                hits += 1
            else:
                score = m.vis_px_s(axis)
                if score < need * 0.45:
                    hits += 1
                else:
                    hits = 0
            if hits >= 10:
                return
            time.sleep(0.05)

    def velocity_sweep(self, axis: str, speeds: Optional[list[float]] = None) -> list[SweepPoint]:
        speeds = speeds or SWEEP_SPEEDS[axis]
        st = self._status()
        self._capture_origin(st)
        if not st.homed:
            self.log(f"{axis} sweep without HOME — staying conservative and aborting on fault")
        budget = TUNER_TRAVEL_BUDGET[axis]
        self.log(f"{axis} cable-safe window {budget} steps from {st.pos(axis):.0f}")
        direction = self._travel_dir(axis, st)
        hold = SWEEP_MEASURE_S.get(axis, 0.70)
        out: list[SweepPoint] = []
        try:
            for speed in speeds:
                self._abort_if()
                st0 = self._status(zoom_safety=(axis == "zoom"))
                if self._over_budget(axis, st0):
                    self.log(f"{axis} sweep stopped at travel cap")
                    break
                burst = min(hold, 0.55 * budget / max(speed, 1.0))
                burst = max(0.35, burst)
                self.rec.trial_name = f"sweep-{axis}-{speed}"
                self.log(f"{axis} sweep {speed:g} step/s  burst {burst:.2f}s then wait for NDI")
                self.ctl.stop()
                self._sleep(0.2)
                st0 = self._status(zoom_safety=(axis == "zoom"))
                t_cmd = time.perf_counter()
                try:
                    self._vel(axis, direction * speed)
                    self._sleep(burst, zoom_safety=(axis == "zoom"), axis=axis)
                except TrialAborted as e:
                    self.ctl.stop()
                    if "travel cap" in str(e):
                        self.log(str(e))
                        break
                    raise
                finally:
                    self.ctl.stop()
                st1 = self._status(zoom_safety=(axis == "zoom"))
                steps = abs(st1.pos(axis) - st0.pos(axis))
                t_vis = self.wait_for_image_motion(t_cmd, timeout=6.0, axis=axis)
                if t_vis <= 0:
                    self.log(
                        f"  NDI never showed this move (waited 6s). "
                        f"OBS delay may be huge, or the frame is frozen. steps={steps:.0f}"
                    )
                    meas = {
                        "mean_px_s": 0.0,
                        "mean_mag": 0.0,
                        "mean_jitter": 0.0,
                        "mean_radial": 0.0,
                        "still_frac": 1.0,
                        "low_texture": True,
                        "steps": steps,
                    }
                    pt = self._point(axis, speed, direction, t_cmd, time.perf_counter(), meas, "ndi_timeout")
                    out.append(pt)
                    self.report.sweep.append(pt)
                    self.wait_for_image_still(2.0, axis=axis)
                    self._return_to_origin(axis)
                    continue
                t0 = t_vis
                self._sleep(0.85)
                t1 = time.perf_counter()
                meas = self._measure_window(t0, t1, axis)
                meas["steps"] = steps
                pt = self._point(axis, speed, direction, t0, t1, meas, "")
                if axis == "zoom" and st1.pwm_hit:
                    pt.aborted = "pwm_hit"
                out.append(pt)
                self.report.sweep.append(pt)
                kind = "zoom-scale" if axis == "zoom" else "slide"
                self.log(
                    f"  NDI lag {self.report.latency_s*1000:.0f} ms  "
                    f"{kind} {pt.mean_px_s:.2f} px/s  px/step={pt.px_per_step:.4f}  "
                    f"jitter={pt.jitter_ratio:.2f}  still={pt.still_frac:.0%}"
                    + ("  NOT SMOOTH" if not pt.is_smooth(
                        self.report.stillness_zoom_px_s if axis == "zoom"
                        else self.report.stillness_floor_px_s
                    ) else "")
                )
                self.wait_for_image_still(3.5, axis=axis)
                self._return_to_origin(axis)
                self.wait_for_image_still(3.5, axis=axis)
                if len(out) >= 2 and out[-1].steps_moved < 8 and out[-2].steps_moved < 8:
                    self.log(f"{axis} sweep stopped — firmware position is not changing")
                    break
                if axis == "zoom" and st1.pwm_hit:
                    self.log("zoom PWM rubber seen — stopping sweep")
                    break
        finally:
            self.ctl.stop()
            self._return_to_origin(axis)
        return out

    def _point(self, axis, speed, direction, t0, t1, meas, aborted) -> SweepPoint:
        dur = max(t1 - t0, 1e-3)
        steps = float(meas["steps"])
        px_per_step = (meas["mean_px_s"] * dur / steps) if steps > 2 else 0.0
        mag = max(meas["mean_mag"], 1e-4)
        return SweepPoint(
            axis=axis,
            speed=float(speed),
            direction=float(direction),
            steps_moved=steps,
            duration_s=dur,
            mean_px_s=float(meas["mean_px_s"]),
            mean_mag=float(meas["mean_mag"]),
            mean_jitter=float(meas["mean_jitter"]),
            mean_radial=float(meas["mean_radial"]),
            px_per_step=px_per_step,
            jitter_ratio=float(meas["mean_jitter"]) / mag,
            still_frac=float(meas["still_frac"]),
            low_texture=bool(meas["low_texture"]),
            aborted=aborted,
        )

    def _update_latency(self, t_cmd: float) -> None:
        floor_px_s = max(self.report.stillness_floor_px_s, 0.25)
        need = max(0.8, floor_px_s * 3.0)
        for s in self.rec.since(t_cmd):
            if s.vis_px_s() > need:
                lat = s.t - t_cmd
                if 0.02 < lat < 6.0:
                    if self.report.latency_s <= 0:
                        self.report.latency_s = lat
                    else:
                        self.report.latency_s = 0.6 * self.report.latency_s + 0.4 * lat
                return

    def range_crawl(self, axis: str, speed: float = 0.0) -> RangeResult:
        if speed <= 0:
            speed = 180.0 if axis != "zoom" else 80.0
            speed = min(speed, AXIS_MAX_VEL[axis])
        st = self._status()
        if not st.homed:
            raise TrialAborted("HOME first for range crawl")
        if axis in ("pan", "tilt") and abs(st.pos(axis)) > 2500:
            raise TrialAborted(
                f"{axis} is already {st.pos(axis):.0f} from home — jog back before a range crawl"
            )
        self._capture_origin(st)
        budget = TUNER_TRAVEL_BUDGET[axis]
        direction = TRAVEL_SIGN[axis]
        self.rec.trial_name = f"range-{axis}"
        self.log(f"{axis} range crawl {speed:g} step/s, cap {budget} steps (cables)")
        start = st.pos(axis)
        t0 = time.perf_counter()
        timeout = (budget / max(speed, 1.0)) + 4.0
        self._vel(axis, direction * speed)
        still_hits = 0
        saw_motion = False
        reason = "timeout"
        samples = 0
        floor_px_s = max(self.report.stillness_floor_px_s, 0.25)
        motion_need = max(0.8, floor_px_s * 3.0)
        if axis == "zoom":
            motion_need = self._motion_need("zoom")
        try:
            while time.perf_counter() - t0 < timeout:
                self._abort_if()
                st = self._status(zoom_safety=(axis == "zoom"))
                samples += 1
                if st.any_fault:
                    reason = "fault"
                    break
                if axis == "zoom" and st.pwm_hit:
                    reason = "pwm_hit"
                    break
                if self._over_budget(axis, st):
                    reason = "tuner travel cap"
                    break
                m = self.latest_metrics()
                elapsed = time.perf_counter() - t0
                if m:
                    score = m.vis_px_s(axis)
                    if score >= motion_need:
                        saw_motion = True
                        still_hits = 0
                    elif saw_motion and elapsed > 1.2:
                        if score < motion_need * 0.35:
                            still_hits += 1
                        else:
                            still_hits = 0
                        if still_hits >= 16:
                            reason = "image stopped"
                            break
                time.sleep(0.05)
        finally:
            self.ctl.stop()
        if not saw_motion and reason == "timeout":
            reason = "never saw image motion"
        self._sleep(0.2)
        end = self._status().pos(axis)
        result = RangeResult(axis=axis, start=start, end=end, span=end - start, reason=reason, samples=samples)
        self.report.ranges.append(result)
        self.log(f"{axis} range {start:.0f} → {end:.0f} (span {result.span:.0f}) {reason}")
        self._return_to_origin(axis)
        return result

    def goto_duration(self, index: Optional[int] = None) -> GotoResult:
        presets = self.ctl.presets()
        target = None
        if index is not None:
            for p in presets:
                if int(p.get("index", -1)) == index and p.get("valid"):
                    target = p
                    break
        else:
            for p in presets:
                if int(p.get("index", 0)) >= 1 and p.get("valid"):
                    target = p
                    break
        if not target:
            raise TrialAborted("no valid preset to GOTO")
        idx = int(target["index"])
        name = str(target.get("name") or f"#{idx}")
        dur = float(target.get("duration_s") or 0.0)
        self.rec.trial_name = f"goto-{idx}"
        self.log(f"GOTO {idx} {name!r} duration={dur:.2f}s (briefly enabling automations)")
        prev_recall = bool(self._status().preset_recall)
        self.ctl.set_preset_recall(True)
        t0 = time.perf_counter()
        try:
            resp = self.ctl.goto_preset(idx)
            if str(resp.get("status")) != "ok":
                err = str(resp.get("error") or "GOTO refused")
                raise TrialAborted(err)
            firmware_end = None
            seen_moving = False
            timeout = max(dur, 8.0) + 20.0
            while time.perf_counter() - t0 < timeout:
                self._abort_if()
                st = self._status()
                if st.moving:
                    seen_moving = True
                    firmware_end = None
                elif seen_moving:
                    firmware_end = time.perf_counter() - t0
                    self._sleep(0.8)
                    break
                time.sleep(0.05)
        finally:
            self.ctl.set_preset_recall(prev_recall)
            self.ctl.stop()
        chunk = self.rec.since(t0)
        floor_px_s = max(self.report.stillness_floor_px_s, 0.25)
        need = max(0.8, floor_px_s * 3.0)
        visual_start = 0.0
        visual_end = 0.0
        peak = 0.0
        moving = False
        for s in chunk:
            peak = max(peak, s.vis_px_s())
            rel = s.t - t0
            if not moving and s.vis_px_s() > need:
                moving = True
                visual_start = rel
            if moving and s.vis_px_s() < need * 0.4:
                visual_end = rel
                break
        if visual_end <= 0 and chunk:
            visual_end = chunk[-1].t - t0
        fw = firmware_end or visual_end
        result = GotoResult(
            index=idx,
            name=name,
            duration_s=dur,
            visual_start_s=visual_start,
            visual_end_s=visual_end,
            firmware_end_s=fw,
            overshoot_s=max(0.0, visual_end - fw),
            latency_s=visual_start,
            peak_px_s=peak,
        )
        self.report.goto = result
        self.report.latency_s = visual_start or self.report.latency_s
        self.log(
            f"GOTO visual {visual_start:.2f}–{visual_end:.2f}s  firmware {fw:.2f}s  overshoot {result.overshoot_s:.2f}s"
        )
        return result

    def irun_search(self, axis: str = "pan", speed: float = 450.0) -> list[IrunPoint]:
        st = self._status()
        key = f"{axis}_irun"
        original = int(getattr(st, key))
        self._capture_origin(st)
        self.log("IRUN: wait for NDI to go still (GOTO delay), then one ignored warmup burst")
        self.wait_for_image_still(5.0, axis=axis)
        self._ndi_burst(axis, speed)
        self.wait_for_image_still(3.0, axis=axis)
        lo = max(3, original - 3)
        hi = min(16, original + 1)
        points: list[IrunPoint] = []
        try:
            for cs in range(lo, hi + 1):
                self._abort_if()
                self.log(f"{axis} IRUN CS {cs} @ {speed:g} step/s (burst then wait for NDI)")
                self.ctl.set_irun(**{key: cs})
                self.ctl.stop()
                self._sleep(0.25)
                meas = self._ndi_burst(axis, speed)
                steps = float(meas.get("steps") or 0.0)
                mag = max(float(meas.get("mean_mag") or 0.0), 1e-4)
                pt = IrunPoint(
                    axis=axis,
                    cs=cs,
                    speed=speed,
                    px_per_step=(meas["mean_px_s"] * 0.85 / steps) if steps > 2 else 0.0,
                    jitter_ratio=float(meas.get("mean_jitter") or 0.0) / mag,
                    mean_px_s=float(meas["mean_px_s"]),
                    original=(cs == original),
                )
                points.append(pt)
                self.report.irun.append(pt)
                self.log(
                    f"  CS {cs}: {pt.mean_px_s:.2f} px/s  px/step={pt.px_per_step:.4f}"
                    + ("  (timeout)" if meas.get("aborted") else "")
                )
        finally:
            self.ctl.stop()
            try:
                self.ctl.set_irun(**{key: original})
                self.log(f"restored {axis} IRUN CS {original}")
            except Exception as e:
                self.log(f"WARNING: could not restore IRUN: {e}")
            try:
                self._return_to_origin(axis)
            except TrialAborted:
                self.ctl.stop()
        return points

    def zoom_pt_scale_check(self, pan_speed: float = 450.0) -> ZoomPtScaleResult:
        """Pan the same command at two zoom fractions; match on-screen speed."""
        st = self._status()
        self._capture_origin(st)
        span = st.zoom_span if (st.zoom_cal_valid and 120.0 < st.zoom_span < 3500.0) else 0.0
        if span <= 80.0 and st.zoom_soft_max > 80.0:
            span = st.zoom_soft_max
        if span <= 80.0:
            span = max(st.zoom + 400.0, 400.0)
            self.log("Tele pan scale: no saved zoom span — z fractions will be rough")
        cur_min = float(FIRMWARE["ZOOM_PT_SCALE_MIN"])
        z_now = min(max(st.zoom / span, 0.0), 1.0)
        room = TUNER_TRAVEL_BUDGET["zoom"] - 40.0
        if st.zoom_soft_max > st.zoom + 80:
            room = min(900.0, st.zoom_soft_max - st.zoom - 40.0)
        travel = min(room, 0.45 * span)
        travel = max(travel, 80.0)
        toward_tele = z_now < 0.55
        self.log(
            f"Tele pan scale: pan {pan_speed:g} at two zooms, "
            f"span={span:.0f} z={z_now:.2f} travel={travel:.0f} toward {'tele' if toward_tele else 'wide'}"
        )

        def sample(tag: str) -> tuple[float, float]:
            st_i = self._status()
            z = min(max(st_i.zoom / span, 0.0), 1.0)
            meas = self._ndi_burst("pan", pan_speed)
            px = float(meas["mean_px_s"])
            self.log(f"  {tag} z={z:.2f}  pan {px:.2f} px/s  {meas.get('aborted') or 'ok'}")
            return z, px

        z0, px0 = sample("near")
        self.ctl.stop()
        zoom_dir = TRAVEL_SIGN["zoom"] if toward_tele else -TRAVEL_SIGN["zoom"]
        st_z = self._status()
        cap = st_z.zoom + zoom_dir * travel
        if toward_tele and st_z.zoom_soft_max > 80:
            cap = min(cap, st_z.zoom_soft_max - 24)
        if (not toward_tele) and st_z.zoom_soft_min >= 0:
            cap = max(cap, st_z.zoom_soft_min + 24)
        self._vel("zoom", zoom_dir * 80.0)
        t0 = time.perf_counter()
        try:
            while time.perf_counter() - t0 < abs(travel) / 80.0 + 2.5:
                self._abort_if()
                st_z = self._status(zoom_safety=True)
                if toward_tele and st_z.zoom >= cap:
                    break
                if (not toward_tele) and st_z.zoom <= cap:
                    break
                if st_z.pwm_hit:
                    break
                time.sleep(0.05)
        finally:
            self.ctl.stop()
        self.wait_for_image_still(2.5, axis="zoom")
        z1, px1 = sample("far")
        self._return_to_origin("zoom")
        note = ""
        if abs(z1 - z0) < 0.12:
            note = "Need a larger zoom change (or a saved zoom span) for a stronger reading."
        result = ZoomPtScaleResult(
            z0=z0, z1=z1, px0=px0, px1=px1, current_min=cur_min, suggested_min=cur_min, note=note
        )
        rec, why = suggest_zoom_pt_scale(result)
        result.suggested_min = rec
        result.note = why if not note else f"{note} {why}"
        self.report.zoom_pt_scale = result
        self.log(f"ZOOM_PT_SCALE_MIN {why}")
        return result

    def full_suite(self, do_range: bool = True, do_goto: bool = True) -> TunerReport:
        self.report = TunerReport()
        self.ctl.mute_automations()
        self.report.notes.append("Preset automations were muted for the suite.")
        try:
            st = self._status()
            if not st.homed:
                self.wait_home()
            self.stillness_baseline()
            for axis in ("pan", "tilt", "zoom"):
                self.velocity_sweep(axis)
                self.wait_still(0.8)
            if do_range:
                self.log("Range crawl: zoom only in the full suite (pan/tilt stay in the cable-safe window).")
                try:
                    self.range_crawl("zoom")
                except TrialAborted as e:
                    self.log(f"range zoom skipped: {e}")
                    self.report.notes.append(f"range zoom: {e}")
                self.wait_still(0.6)
                self.report.notes.append(
                    "Pan/tilt range crawl skipped in the full suite so cabling cannot wrap. "
                    "Sweep pan/tilt stays within 700/900 steps of the start pose and returns after each speed."
                )
            if do_goto:
                try:
                    self.goto_duration()
                except TrialAborted as e:
                    self.log(f"GOTO skipped: {e}")
                    self.report.notes.append(f"GOTO: {e}")
            self.wait_for_image_still(5.0)
            try:
                self.irun_search("pan", 450.0)
            except TrialAborted as e:
                self.log(f"IRUN skipped: {e}")
                self.report.notes.append(f"IRUN: {e}")
            self.wait_still(0.5)
            try:
                self.zoom_pt_scale_check(450.0)
            except TrialAborted as e:
                self.log(f"tele pan scale skipped: {e}")
                self.report.notes.append(f"ZOOM_PT_SCALE: {e}")
        finally:
            self.ctl.stop()
        return self.report
