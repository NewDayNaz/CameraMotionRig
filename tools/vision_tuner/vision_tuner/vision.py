from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


PROC_WIDTH = 640
TEXTURE_MIN = 4.0
NOISE_PX_S = 0.25


@dataclass
class VisionMetrics:
    flow_dx: float = 0.0
    flow_dy: float = 0.0
    flow_mag: float = 0.0
    jitter: float = 0.0
    radial: float = 0.0
    zoom_px_s: float = 0.0
    sharpness: float = 0.0
    texture: float = 0.0
    low_texture: bool = True
    dt: float = 0.0
    px_per_s: float = 0.0
    still: bool = True

    def vis_px_s(self, axis: str = "") -> float:
        if axis == "zoom":
            return self.zoom_px_s
        if axis in ("pan", "tilt"):
            return self.px_per_s
        return max(self.px_per_s, self.zoom_px_s)


class VisionEngine:
    def __init__(self):
        self._prev_gray: Optional[np.ndarray] = None
        self._last_t: Optional[float] = None
        self._grid: Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None

    def reset(self) -> None:
        self._prev_gray = None
        self._last_t = None
        self._grid = None

    def process(self, bgr: np.ndarray, t: float) -> VisionMetrics:
        gray_full = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        h0, w0 = gray_full.shape
        scale = PROC_WIDTH / float(max(w0, 1))
        if scale < 1.0:
            gray = cv2.resize(gray_full, (int(w0 * scale), int(h0 * scale)), interpolation=cv2.INTER_AREA)
        else:
            gray = gray_full
        h, w = gray.shape
        texture = float(np.mean(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3) ** 2))
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        m = VisionMetrics(sharpness=sharpness, texture=texture, low_texture=texture < TEXTURE_MIN)

        if self._prev_gray is None or self._prev_gray.shape != gray.shape or self._last_t is None:
            self._prev_gray = gray
            self._last_t = t
            return m

        dt = max(t - self._last_t, 1e-3)
        flow = cv2.calcOpticalFlowFarneback(
            self._prev_gray,
            gray,
            None,
            0.5,
            3,
            21,
            3,
            5,
            1.2,
            0,
        )
        fx = flow[..., 0]
        fy = flow[..., 1]
        mag = np.sqrt(fx * fx + fy * fy)
        # Ignore a thin border; Farneback is noisy there.
        y0, y1 = h // 8, h - h // 8
        x0, x1 = w // 8, w - w // 8
        crop_fx = fx[y0:y1, x0:x1]
        crop_fy = fy[y0:y1, x0:x1]
        crop_mag = mag[y0:y1, x0:x1]
        med_x = float(np.median(crop_fx))
        med_y = float(np.median(crop_fy))
        med_mag = float(np.hypot(med_x, med_y))
        residual = crop_mag - med_mag
        jitter = float(np.median(np.abs(residual)))

        if self._grid is None or self._grid[0].shape != gray.shape:
            yy, xx = np.mgrid[0:h, 0:w]
            cx, cy = (w - 1) * 0.5, (h - 1) * 0.5
            rx, ry = xx - cx, yy - cy
            r = np.sqrt(rx * rx + ry * ry) + 1e-6
            self._grid = (rx / r, ry / r, r, (r > min(w, h) * 0.15) & (r < min(w, h) * 0.48))
        ux, uy, _r, mask = self._grid
        rad_field = fx * ux + fy * uy
        radial_signed = float(np.median(rad_field[mask]))
        radial_abs = float(np.mean(np.abs(rad_field[mask])))

        to_full = (w0 / float(w)) if w else 1.0
        m.flow_dx = med_x * to_full
        m.flow_dy = med_y * to_full
        m.flow_mag = med_mag * to_full
        m.jitter = jitter * to_full
        m.radial = radial_signed * to_full
        m.dt = dt
        m.px_per_s = m.flow_mag / dt
        # Zoom is scale: pixels slide along rays from the center. Translation
        # (px_per_s) stays near 0 even when the lens is moving.
        m.zoom_px_s = (radial_abs * to_full) / dt
        m.still = m.px_per_s < NOISE_PX_S and m.zoom_px_s < NOISE_PX_S
        self._prev_gray = gray
        self._last_t = t
        return m


def overlay(bgr: np.ndarray, metrics: VisionMetrics, hud: dict) -> np.ndarray:
    vis = bgr.copy()
    h, w = vis.shape[:2]
    cx, cy = w // 2, h // 2
    # Pan/tilt: translation arrow. Zoom: expanding/contracting ring.
    vx = int(np.clip(metrics.flow_dx * 12.0, -w // 3, w // 3))
    vy = int(np.clip(metrics.flow_dy * 12.0, -h // 3, h // 3))
    color = (40, 220, 80) if not metrics.still else (80, 80, 80)
    if metrics.low_texture:
        color = (40, 180, 255)
    cv2.arrowedLine(vis, (cx, cy), (cx + vx, cy + vy), color, 2, tipLength=0.2)
    cv2.circle(vis, (cx, cy), 4, color, -1)
    zoom_r = int(np.clip(40 + metrics.zoom_px_s * 4.0, 18, min(w, h) // 4))
    zoom_color = (40, 200, 255) if metrics.zoom_px_s >= NOISE_PX_S else (70, 70, 70)
    cv2.circle(vis, (cx, cy), zoom_r, zoom_color, 1)
    zdir = "tele" if metrics.radial < -1e-4 else ("wide" if metrics.radial > 1e-4 else "—")
    lines = [
        f"pan/tilt {metrics.flow_mag:.3f}px  {metrics.px_per_s:.2f} px/s",
        f"zoom {metrics.zoom_px_s:.2f} px/s  {zdir}  radial {metrics.radial:+.3f}",
        f"jitter {metrics.jitter:.3f}  sharp {metrics.sharpness:.0f}  tex {metrics.texture:.1f}",
    ]
    if metrics.low_texture:
        lines.append("LOW TEXTURE — point at a detailed scene")
    for k, v in hud.items():
        lines.append(f"{k}: {v}")
    y = 22
    for line in lines:
        cv2.putText(vis, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
        y += 20
    return vis
