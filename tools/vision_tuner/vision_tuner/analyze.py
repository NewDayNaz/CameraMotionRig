from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from vision_tuner.config import AXIS_MAX_RANGE, AXIS_MAX_VEL, FIRMWARE


def pt_scale_at(z: float, scale_min: float) -> float:
    z = min(max(z, 0.0), 1.0)
    return 1.0 - z * (1.0 - scale_min)


@dataclass
class SweepPoint:
    axis: str
    speed: float
    direction: float
    steps_moved: float
    duration_s: float
    mean_px_s: float
    mean_mag: float
    mean_jitter: float
    mean_radial: float
    px_per_step: float
    jitter_ratio: float
    still_frac: float
    low_texture: bool
    aborted: str = ""

    def is_usable(self, floor_px_s: float) -> bool:
        if self.aborted:
            return False
        if self.steps_moved < 8:
            return False
        noise = max(min(floor_px_s, 0.08), 0.02)
        min_signal = 0.18 if self.axis == "zoom" else 0.3
        return self.mean_px_s >= max(min_signal, 3.0 * noise)

    def is_smooth(self, floor_px_s: float) -> bool:
        # Far-field NDI: jitter ratio is noise/signal and is not a smoothness score.
        return self.is_usable(floor_px_s)

    @property
    def smooth(self) -> bool:
        return self.is_smooth(0.25)


@dataclass
class RangeResult:
    axis: str
    start: float
    end: float
    span: float
    reason: str
    samples: int = 0


@dataclass
class GotoResult:
    index: int
    name: str
    duration_s: float
    visual_start_s: float
    visual_end_s: float
    firmware_end_s: float
    overshoot_s: float
    latency_s: float
    peak_px_s: float
    error: str = ""


@dataclass
class IrunPoint:
    axis: str
    cs: int
    speed: float
    px_per_step: float
    jitter_ratio: float
    mean_px_s: float
    original: bool = False


@dataclass
class ZoomPtScaleResult:
    z0: float
    z1: float
    px0: float
    px1: float
    current_min: float
    suggested_min: float
    note: str = ""


@dataclass
class TunerReport:
    stillness_floor: float = 0.0
    stillness_floor_px_s: float = 0.0
    stillness_zoom_px_s: float = 0.0
    latency_s: float = 0.0
    sweep: list[SweepPoint] = field(default_factory=list)
    ranges: list[RangeResult] = field(default_factory=list)
    goto: Optional[GotoResult] = None
    irun: list[IrunPoint] = field(default_factory=list)
    zoom_pt_scale: Optional[ZoomPtScaleResult] = None
    notes: list[str] = field(default_factory=list)

    def suggestions_text(self) -> str:
        return format_suggestions(self)


def pick_max_smooth_speed(points: list[SweepPoint], axis: str, floor_px_s: float = 0.25) -> Optional[SweepPoint]:
    """Fastest usable speed, unless image speed collapses at the top of the sweep.

    px/step on a delayed NDI burst is not lost-steps: slow moves spend more of the
    visual window in motion, so the ratio is often *higher* at 40–80 step/s.
    """
    axis_pts = [
        p for p in points
        if p.axis == axis and not p.aborted and p.is_smooth(floor_px_s)
    ]
    if not axis_pts:
        return None
    ordered = sorted(axis_pts, key=lambda x: x.speed)
    peak = max(p.mean_px_s for p in ordered)
    if peak <= 0:
        return ordered[-1]
    collapse = 0.50 * peak
    i = len(ordered) - 1
    while i > 0 and ordered[i].mean_px_s < collapse:
        i -= 1
    return ordered[i]


def pick_irun(
    points: list[IrunPoint], floor_px_s: float = 0.0
) -> tuple[Optional[IrunPoint], str]:
    """Keep original CS unless a *lower* current still matches it.

    Lost steps lower image speed. A first-burst spike (GOTO still on NDI)
    looks like 'CS 8 is better' and must not win. Bursts that sit on the
    stillness floor are noise, not torque.
    """
    if not points:
        return None, "no IRUN samples"
    orig = next((p for p in points if p.original), None)
    ordered = sorted(points, key=lambda p: p.cs)
    if orig is None:
        orig = ordered[-1]
    need = max(1.2, 2.5 * max(floor_px_s, 0.05))
    if orig.mean_px_s < need:
        return orig, (
            f"keep CS {orig.cs} — IRUN bursts ({orig.mean_px_s:.1f} px/s) were not "
            f"clearly above stillness ({floor_px_s:.1f} px/s). Leave Motor calibration."
        )
    cluster = [p.mean_px_s for p in ordered if p.mean_px_s >= 0.4]
    if len(cluster) >= 3:
        xs = sorted(cluster)
        mid = xs[len(xs) // 2]
        if mid < need:
            return orig, (
                f"keep CS {orig.cs} — IRUN cluster ~{mid:.1f} px/s is stillness/NDI noise"
            )
    if orig.mean_px_s < 0.4:
        return orig, "keep — NDI too weak at the current CS"
    if len(ordered) >= 4 and ordered[0].mean_px_s > 1.45 * ordered[-1].mean_px_s:
        return orig, (
            f"keep CS {orig.cs} — image speed fell as CS rose "
            f"({ordered[0].mean_px_s:.1f} → {ordered[-1].mean_px_s:.1f}); "
            "that is leftover NDI from GOTO, not extra torque at low current"
        )
    hi = [p for p in ordered if p.cs >= orig.cs - 1 and p.mean_px_s >= 0.4]
    ref = orig.mean_px_s
    if hi:
        xs = sorted(p.mean_px_s for p in hi)
        ref = xs[len(xs) // 2]
    lo = orig.cs
    pick = orig
    for p in ordered:
        if p.cs >= orig.cs:
            continue
        if p.mean_px_s > 1.35 * max(ref, orig.mean_px_s):
            continue
        if p.mean_px_s < 0.75 * ref:
            continue
        if p.cs < lo:
            lo = p.cs
            pick = p
    if pick.cs == orig.cs:
        return orig, f"keep CS {orig.cs} — lower currents did not match this image speed"
    return pick, (
        f"lowest CS that still matches CS {orig.cs} is {pick.cs} "
        f"({pick.mean_px_s:.1f} vs {orig.mean_px_s:.1f} px/s). Apply on the ESP32 if you agree."
    )


def suggest_zoom_pt_scale(res: ZoomPtScaleResult) -> tuple[float, str]:
    cur = res.current_min
    if res.px0 < 0.4 or res.px1 < 0.4:
        return cur, "keep — NDI too weak at one zoom to retune ZOOM_PT_SCALE_MIN"
    dz = abs(res.z1 - res.z0)
    if dz < 0.12:
        return cur, "keep — zoom travel was too small to measure tele vs wide pan"
    s0 = pt_scale_at(res.z0, cur)
    s1 = pt_scale_at(res.z1, cur)
    if s0 < 0.05:
        return cur, "keep — zoom fraction invalid"
    q = (s1 / s0) * (res.px0 / max(res.px1, 1e-6))
    denom = q * res.z0 - res.z1
    if abs(denom) < 1e-4:
        return cur, "keep — could not fit a scale curve"
    m = (q - 1.0) / denom
    suggested = min(0.85, max(0.25, 1.0 - m))
    if abs(suggested - cur) < 0.08:
        return cur, (
            f"keep {cur:g} — on-screen pan at z={res.z0:.2f} was {res.px0:.2f} px/s, "
            f"at z={res.z1:.2f} was {res.px1:.2f} px/s (already close)"
        )
    tag = "lower" if suggested < cur else "raise"
    return round(suggested, 2), (
        f"{tag} toward {suggested:.2f} so tele pan matches wide on the NDI picture "
        f"(now {cur:g}; z {res.z0:.2f}@{res.px0:.2f} px/s vs z {res.z1:.2f}@{res.px1:.2f} px/s)"
    )


def np_median(vals: list[float]) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def format_suggestions(report: TunerReport) -> str:
    lines = [
        "Vision tuner suggestions — review before changing firmware.",
        "These are measured from NDI/optical flow, not from the TMC UART.",
        "",
        f"Stillness floor: slide {report.stillness_floor:.4f} px/frame ({report.stillness_floor_px_s:.4f} px/s)  "
        f"zoom-scale {report.stillness_zoom_px_s:.4f} px/s",
        f"NDI/motion latency (flow rise after command): {report.latency_s*1000:.0f} ms",
        "",
    ]
    if (
        not report.sweep
        and not report.ranges
        and report.goto is None
        and not report.irun
        and report.zoom_pt_scale is None
    ):
        if report.stillness_floor_px_s < 0.05:
            lines.append("Stillness is clean — the NDI picture is not drifting. Run Sweep pan next.")
        else:
            lines.append("Stillness measured. If the HUD still shows motion, wait for the scene to settle and run Stillness again.")
        lines.append("")
        lines.append("Do not auto-flash these. Copy the ones you agree with into stepper_limits.h or Motor calibration.")
        return "\n".join(lines)

    floor_px_s = max(min(report.stillness_floor_px_s, 0.08), 0.02)
    if report.stillness_floor_px_s > 4.0:
        lines.append("WARNING: scene is not still, or NDI is noisy. Point at architecture, not a black field.")
        lines.append("")
    elif report.stillness_floor_px_s > 0.15:
        lines.append(
            "NOTE: stillness was not a dead hold — people or NDI noise will inflate IRUN/GOTO numbers."
        )
        lines.append("")

    for axis in ("pan", "tilt", "zoom"):
        ax_floor = floor_px_s
        if axis == "zoom":
            ax_floor = max(min(report.stillness_zoom_px_s or 0.02, 0.12), 0.02)
        best = pick_max_smooth_speed(report.sweep, axis, ax_floor)
        cur = AXIS_MAX_VEL[axis]
        key = {
            "pan": "MAX_PAN_VELOCITY",
            "tilt": "MAX_TILT_VELOCITY",
            "zoom": "MAX_ZOOM_VELOCITY",
        }[axis]
        pts = [p for p in report.sweep if p.axis == axis]
        if not pts:
            lines.append(f"{key:24}  (no sweep)")
            continue
        if best is None:
            timed_out = all(p.aborted == "ndi_timeout" for p in pts) if pts else False
            extra = " Slow bursts may be below NDI noise; that is not a firmware speed limit."
            lines.append(
                f"{key:24}  keep {cur}  — no usable NDI motion on this axis.{extra if timed_out else ''}"
            )
            continue
        rec = round(best.speed * 0.92, 1)
        if best.speed >= cur * 0.9 or rec >= cur * 0.9:
            rec = cur
            tag = "keep"
        else:
            tag = "lower" if rec < cur - 5 else "raise" if rec > cur + 5 else "keep"
        lines.append(
            f"{key:24}  {rec:<8}  now {cur}  ({tag})  "
            f"last clean {best.speed:g} step/s  {best.mean_px_s:.2f} px/s  "
            f"px/step={best.px_per_step:.4f}  jitter={best.jitter_ratio:.2f}"
        )
        later_fail = next(
            (
                p for p in pts
                if p.speed > best.speed and not p.aborted and not p.is_smooth(ax_floor)
            ),
            None,
        )
        if later_fail:
            lines.append(
                f"{'':24}  smoothness dropped at {later_fail.speed:g} "
                f"(px/s={later_fail.mean_px_s:.2f}, px/step={later_fail.px_per_step:.4f})"
            )

    tilt_pts = [p for p in report.sweep if p.axis == "tilt"]
    if tilt_pts:
        tilt_to = any(p.aborted == "ndi_timeout" for p in tilt_pts)
        tilt_ok = any(not p.aborted and p.mean_px_s >= 0.3 for p in tilt_pts)
        if tilt_to and tilt_ok:
            lines.append(
                "Slow tilt timeouts on a far sanctuary shot are expected — "
                "that is not a reason to lower MAX_TILT_VELOCITY."
            )

    zoom_pts = [p for p in report.sweep if p.axis == "zoom" and p.is_smooth(floor_px_s)]
    if zoom_pts:
        slow = min(zoom_pts, key=lambda p: p.speed)
        if slow.speed <= FIRMWARE["MIN_ZOOM_VELOCITY"] + 1:
            lines.append(
                f"{'MIN_ZOOM_VELOCITY':24}  keep {FIRMWARE['MIN_ZOOM_VELOCITY']}  "
                f"(70 looked usable; jitter={slow.jitter_ratio:.2f})"
            )
        else:
            lines.append(
                f"{'MIN_ZOOM_VELOCITY':24}  {slow.speed:<8}  70 was not clean — do not ease below this"
            )

    pt = pick_max_smooth_speed(report.sweep, "pan", floor_px_s)
    if pt:
        cur_pt = FIRMWARE["PRESET_PAN_TILT_VELOCITY"]
        if pt.speed >= AXIS_MAX_VEL["pan"] * 0.9:
            lines.append(
                f"{'PRESET_PAN_TILT_VELOCITY':24}  keep {cur_pt}  "
                "(pan sweep did not collapse at firmware max)"
            )
        else:
            rec_pt = min(pt.speed * 0.35, cur_pt * 1.2)
            rec_pt = max(40.0, round(rec_pt, 1))
            lines.append(
                f"{'PRESET_PAN_TILT_VELOCITY':24}  {rec_pt:<8}  now {cur_pt}  "
                "(cruise ~1/3 of last clean pan, for duration=0 recalls)"
            )

    lines.append("")
    latest_range = {}
    for r in report.ranges:
        latest_range[r.axis] = r
    for axis in ("pan", "tilt", "zoom"):
        r = latest_range.get(axis)
        if r is None:
            continue
        key = {
            "pan": "MAX_PAN_RANGE_STEPS",
            "tilt": "MAX_TILT_RANGE_STEPS",
            "zoom": "MAX_ZOOM_RANGE_STEPS",
        }[axis]
        cur = AXIS_MAX_RANGE[axis]
        rec = int(abs(r.span))
        bogus = (
            r.reason in ("never saw image motion", "tuner travel cap")
            or rec < max(800, int(cur * 0.15))
        )
        if bogus:
            lines.append(
                f"{key:24}  keep {cur}  measured span {rec} ({r.reason}) "
                f"start={r.start:.0f} end={r.end:.0f} — not a hard stop, ignore this span."
            )
        else:
            lines.append(
                f"{key:24}  {rec:<8}  now {cur}  start={r.start:.0f} end={r.end:.0f}  ({r.reason})"
            )

    if report.goto:
        g = report.goto
        lines.append("")
        lines.append(
            f"GOTO {g.index} {g.name!r} duration={g.duration_s:.2f}s  "
            f"visual={g.visual_end_s:.2f}s  firmware={g.firmware_end_s:.2f}s  "
            f"overshoot={g.overshoot_s*1000:.0f}ms  latency={g.latency_s*1000:.0f}ms"
        )
        if g.error:
            lines.append(f"  GOTO error: {g.error}")
        if g.overshoot_s > 0.25:
            if g.overshoot_s <= (g.latency_s or 0) + 0.40:
                lines.append(
                    "  Apparent overshoot is within NDI delay — not evidence the ease-out is wrong."
                )
            else:
                lines.append(
                    "  Image kept moving after firmware MOVING=0 — ease-out may be too abrupt, or NDI is late."
                )
        if g.duration_s > 0.2 and g.visual_end_s + 0.3 < g.duration_s:
            lines.append("  Visual move finished early — one axis (often zoom) likely hit its min speed floor.")

    if report.irun:
        lines.append("")
        lines.append("IRUN search (temporary CS; original current was restored, not saved):")
        for p in report.irun:
            mark = "  (current)" if p.original else ""
            lines.append(
                f"  {p.axis} CS {p.cs}: {p.mean_px_s:.2f} px/s  px/step={p.px_per_step:.4f}  "
                f"jitter={p.jitter_ratio:.2f}{mark}"
            )
        pick, why = pick_irun(report.irun, report.stillness_floor_px_s)
        if pick is None:
            lines.append(f"  IRUN: {why}")
        else:
            lines.append(f"  IRUN: {why}")

    if report.zoom_pt_scale:
        zp = report.zoom_pt_scale
        rec, why = suggest_zoom_pt_scale(zp)
        lines.append("")
        lines.append(f"{'ZOOM_PT_SCALE_MIN':24}  {rec:<8}  now {zp.current_min}  ({why})")
        if zp.note:
            lines.append(f"  {zp.note}")

    if report.notes:
        lines.append("")
        lines.extend(report.notes)

    if report.sweep:
        lines.append("")
        lines.append(
            "Sweep table: axis   speed   px/s  px/step  jitter  still  smooth"
        )
        lines.append("  (zoom px/s is radial scale — FOV expanding/contracting — not a pan slide)")
        lines.append("  (max speed follows last usable image motion; ignore a px/step dip if px/s later recovers)")
        for p in report.sweep:
            ax_floor = floor_px_s
            if p.axis == "zoom":
                ax_floor = max(min(report.stillness_zoom_px_s or 0.02, 0.12), 0.02)
            flag = "yes" if p.is_smooth(ax_floor) else "no"
            lines.append(
                f"  {p.axis:4} {p.speed:7g} {p.mean_px_s:7.2f} {p.px_per_step:8.4f} "
                f"{p.jitter_ratio:6.2f} {p.still_frac:5.0%}  {flag:3} {p.aborted}"
            )

    lines.append("")
    lines.append("Do not auto-flash these. Copy the ones you agree with into stepper_limits.h or Motor calibration.")
    return "\n".join(lines)
