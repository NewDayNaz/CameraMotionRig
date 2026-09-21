from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import cv2
import numpy as np

from vision_tuner.ndi_runtime import ensure_ndi_runtime


FrameCallback = Callable[[np.ndarray, float], None]


def list_ndi_sources() -> tuple[list[str], str]:
    rt = ensure_ndi_runtime()
    names, err = _list_ndi_cyndilib()
    if names:
        return names, ""
    names2, err2 = _list_ndi_ndilib()
    if names2:
        return names2, ""
    if not rt.found:
        return [], rt.error
    if not err:
        return [], (
            f"{rt.summary()}. No NDI sources on the LAN yet — start the camera encoder, "
            "wait a few seconds, Find NDI again. Webcam/Demo still work."
        )
    bits = [rt.summary()] + [x for x in (err, err2) if x]
    return [], " | ".join(bits)


def _list_ndi_cyndilib() -> tuple[list[str], str]:
    ensure_ndi_runtime()
    try:
        from cyndilib.finder import Finder
    except Exception as e:
        return [], f"cyndilib: {e}"
    try:
        finder = Finder()
        finder.open()
        deadline = time.time() + 5.0
        names: list[str] = []
        while time.time() < deadline:
            names = list(finder.get_source_names() or [])
            if names:
                break
            time.sleep(0.25)
        finder.close()
        return names, ""
    except Exception as e:
        return [], f"cyndilib find: {e}"


def _list_ndi_ndilib() -> tuple[list[str], str]:
    ensure_ndi_runtime()
    try:
        import NDIlib as ndi
    except Exception as e:
        return [], f"NDIlib: {e}"
    if not ndi.initialize():
        return [], "NDIlib.initialize() failed"
    find = ndi.find_create_v2()
    if find is None:
        return [], "NDI find_create failed"
    ndi.find_wait_for_sources(find, 2500)
    sources = ndi.find_get_current_sources(find)
    names = [s.ndi_name for s in sources] if sources else []
    ndi.find_destroy(find)
    return names, ""


class VideoSource:
    def read(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def close(self) -> None:
        pass


class WebcamSource(VideoSource):
    def __init__(self, index: int = 0):
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open webcam {index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def read(self) -> Optional[np.ndarray]:
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        self.cap.release()


class DemoSource(VideoSource):
    """Moving checkerboard so the UI can be exercised without a camera."""

    def __init__(self, cmd_fn: Optional[Callable[[], dict]] = None):
        self.cmd_fn = cmd_fn
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0
        self._t = time.perf_counter()
        yy, xx = np.mgrid[0:360, 0:640]
        self._xx = xx.astype(np.float32)
        self._yy = yy.astype(np.float32)

    def read(self) -> Optional[np.ndarray]:
        now = time.perf_counter()
        dt = now - self._t
        self._t = now
        cmd = self.cmd_fn() if self.cmd_fn else {"pan": 0, "tilt": 0, "zoom": 0}
        self.x += float(cmd.get("pan") or 0) * 0.02 * dt
        self.y += float(cmd.get("tilt") or 0) * 0.02 * dt
        self.z *= 1.0 + float(cmd.get("zoom") or 0) * 0.0015 * dt
        self.z = float(np.clip(self.z, 0.5, 2.5))
        xs = (self._xx * self.z + self.x * 8.0).astype(np.int32)
        ys = (self._yy * self.z + self.y * 8.0).astype(np.int32)
        checker = ((xs // 32 + ys // 32) & 1) * 180 + 40
        frame = np.dstack([checker, checker, np.full_like(checker, 70)]).astype(np.uint8)
        cv2.putText(frame, "DEMO SOURCE", (16, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        return frame


class CyndiSource(VideoSource):
    def __init__(self, name: str):
        ensure_ndi_runtime()
        from cyndilib.finder import Finder
        from cyndilib.receiver import Receiver
        from cyndilib.video_frame import VideoFrameSync
        from cyndilib.wrapper.ndi_recv import RecvBandwidth, RecvColorFormat

        self._finder = Finder()
        self._finder.open()
        source = None
        deadline = time.time() + 4.0
        while time.time() < deadline:
            names = list(self._finder.get_source_names() or [])
            match = _pick_name(names, name)
            if match:
                try:
                    with self._finder.notify:
                        source = self._finder.get_source(match)
                except Exception:
                    source = self._finder.get_source(match)
                break
            time.sleep(0.2)
        if source is None:
            self._finder.close()
            raise RuntimeError(f"NDI source not found: {name or '(first)'}")
        self._recv = Receiver(
            color_format=RecvColorFormat.RGBX_RGBA,
            bandwidth=RecvBandwidth.highest,
        )
        self._vf = VideoFrameSync()
        self._recv.frame_sync.set_video_frame(self._vf)
        self._recv.set_source(source)
        for _ in range(40):
            if self._recv.is_connected():
                break
            time.sleep(0.1)
        if not self._recv.is_connected():
            self.close()
            raise RuntimeError("NDI receiver did not connect")

    def read(self) -> Optional[np.ndarray]:
        self._recv.frame_sync.capture_video()
        if min(self._vf.xres, self._vf.yres) == 0:
            return None
        arr = np.asarray(self._vf.get_array())
        frame = arr.reshape(self._vf.yres, self._vf.xres, 4)
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

    def close(self) -> None:
        try:
            if self._recv.is_connected():
                self._recv.disconnect()
        except Exception:
            pass
        try:
            self._finder.close()
        except Exception:
            pass


class NdiLibSource(VideoSource):
    def __init__(self, name: str):
        ensure_ndi_runtime()
        import NDIlib as ndi

        self.ndi = ndi
        if not ndi.initialize():
            raise RuntimeError("NDIlib.initialize() failed")
        find = ndi.find_create_v2()
        if find is None:
            raise RuntimeError("NDI find_create failed")
        match = None
        deadline = time.time() + 4.0
        while time.time() < deadline:
            ndi.find_wait_for_sources(find, 200)
            sources = ndi.find_get_current_sources(find)
            if sources:
                names = [s.ndi_name for s in sources]
                want = _pick_name(names, name)
                if want:
                    match = next(s for s in sources if s.ndi_name == want)
                    break
            time.sleep(0.1)
        if match is None:
            ndi.find_destroy(find)
            raise RuntimeError(f"NDI source not found: {name or '(first)'}")
        create = ndi.RecvCreateV3()
        create.color_format = ndi.RECV_COLOR_FORMAT_BGRX_BGRA
        create.bandwidth = ndi.RECV_BANDWIDTH_HIGHEST
        self.recv = ndi.recv_create_v3(create)
        if self.recv is None:
            ndi.find_destroy(find)
            raise RuntimeError("NDI recv_create failed")
        ndi.recv_connect(self.recv, match)
        ndi.find_destroy(find)

    def read(self) -> Optional[np.ndarray]:
        t, v, _a, _m = self.ndi.recv_capture_v2(self.recv, 40)
        if t != self.ndi.FRAME_TYPE_VIDEO:
            return None
        try:
            frame = np.copy(v.data)
        finally:
            self.ndi.recv_free_video_v2(self.recv, v)
        if frame.ndim == 3 and frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    def close(self) -> None:
        try:
            self.ndi.recv_destroy(self.recv)
        except Exception:
            pass


def open_source(kind: str, ndi_name: str = "", webcam_index: int = 0, cmd_fn=None) -> VideoSource:
    kind = (kind or "ndi").lower()
    if kind == "demo":
        return DemoSource(cmd_fn)
    if kind == "webcam":
        return WebcamSource(webcam_index)
    rt = ensure_ndi_runtime()
    if not rt.found:
        raise RuntimeError(rt.error)
    try:
        return CyndiSource(ndi_name)
    except Exception as first:
        try:
            return NdiLibSource(ndi_name)
        except Exception as second:
            raise RuntimeError(f"NDI open failed ({first}); NDIlib fallback: {second}") from second


def _pick_name(names: list[str], want: str) -> Optional[str]:
    if not names:
        return None
    if not want:
        return names[0]
    want_l = want.lower()
    for n in names:
        if n == want:
            return n
    for n in names:
        if want_l in n.lower():
            return n
    return None


class CaptureThread(threading.Thread):
    def __init__(self, source: VideoSource, on_frame: FrameCallback):
        super().__init__(daemon=True, name="capture")
        self.source = source
        self.on_frame = on_frame
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        prev_small = None
        try:
            while not self._stop.is_set():
                frame = self.source.read()
                if frame is None:
                    time.sleep(0.01)
                    continue
                small = cv2.resize(frame, (80, 45), interpolation=cv2.INTER_AREA)
                if prev_small is not None:
                    delta = float(np.mean(np.abs(small.astype(np.int16) - prev_small)))
                    if delta == 0:
                        continue
                prev_small = small
                self.on_frame(frame, time.perf_counter())
        finally:
            self.source.close()
