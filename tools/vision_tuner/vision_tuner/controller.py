from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional

import requests


@dataclass
class RigStatus:
    pan: float = 0.0
    tilt: float = 0.0
    zoom: float = 0.0
    homed: bool = False
    homing: bool = False
    moving: bool = False
    error: str = ""
    faults: dict = field(default_factory=lambda: {"pan": False, "tilt": False, "zoom": False})
    endstops: dict = field(default_factory=lambda: {"pan": False, "tilt": False, "zoom": False})
    zoom_soft_min: float = 0.0
    zoom_soft_max: float = 0.0
    zoom_cal_valid: bool = False
    zoom_span: float = 0.0
    zoom_pct: Optional[float] = None
    zoom_pwm_thresh: float = 84.0
    pan_irun: int = 11
    tilt_irun: int = 11
    zoom_irun: int = 5
    preset_recall: bool = True
    pwm_hit: bool = False
    sg_result: Optional[float] = None
    tstep: Optional[float] = None
    ok: bool = False
    message: str = ""

    @property
    def any_fault(self) -> bool:
        return any(bool(v) for v in self.faults.values())

    def pos(self, axis: str) -> float:
        return float(getattr(self, axis, 0.0))


class Controller:
    """HTTP client for the ESP32 web API. All calls are serialized."""

    def __init__(self, base_url: str = "http://192.168.1.50", timeout: float = 1.5):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self._lock = threading.Lock()
        self._session = requests.Session()
        self.last_cmd = {"pan": 0.0, "tilt": 0.0, "zoom": 0.0}
        self.last_status = RigStatus()
        self._saved_recall: Optional[bool] = None

    def set_url(self, url: str) -> None:
        self.base_url = url.rstrip("/")

    def _url(self, path: str) -> str:
        return self.base_url + path

    def _request(self, method: str, path: str, **kwargs) -> dict:
        kwargs.setdefault("timeout", self.timeout)
        with self._lock:
            r = self._session.request(method, self._url(path), **kwargs)
        r.raise_for_status()
        if not r.content:
            return {}
        try:
            return r.json()
        except ValueError:
            return {"raw": r.text}

    def ping(self) -> RigStatus:
        st = self.positions()
        if st.ok:
            self.last_status = st
        return st

    def positions(self) -> RigStatus:
        try:
            data = self._request("GET", "/api/positions")
        except requests.RequestException as e:
            st = RigStatus(ok=False, message=str(e))
            self.last_status = st
            return st
        faults = data.get("faults") or {}
        endstops = data.get("endstops") or {}
        st = RigStatus(
            pan=float(data.get("pan") or 0),
            tilt=float(data.get("tilt") or 0),
            zoom=float(data.get("zoom") or 0),
            homed=bool(data.get("homed")),
            homing=bool(data.get("homing")),
            moving=bool(data.get("moving")),
            error=str(data.get("error") or ""),
            faults={
                "pan": bool(faults.get("pan")),
                "tilt": bool(faults.get("tilt")),
                "zoom": bool(faults.get("zoom")),
            },
            endstops={
                "pan": bool(endstops.get("pan")),
                "tilt": bool(endstops.get("tilt")),
                "zoom": bool(endstops.get("zoom")),
            },
            zoom_soft_min=float(data.get("zoom_soft_min") or 0),
            zoom_soft_max=float(data.get("zoom_soft_max") or 0),
            zoom_cal_valid=bool(data.get("zoom_cal_valid")),
            zoom_span=float(data.get("zoom_span") or 0),
            zoom_pct=data.get("zoom_pct"),
            zoom_pwm_thresh=float(data.get("zoom_pwm_thresh") or 84),
            pan_irun=int(data.get("pan_irun") or 11),
            tilt_irun=int(data.get("tilt_irun") or 11),
            zoom_irun=int(data.get("zoom_irun") or 5),
            preset_recall=bool(data.get("preset_recall", True)),
            ok=True,
        )
        self.last_status = st
        return st

    def refresh_zoom_safety(self, st: RigStatus) -> RigStatus:
        try:
            data = self._request("GET", "/api/zoom-cal")
            st.pwm_hit = bool(data.get("pwm_hit"))
        except requests.RequestException:
            pass
        try:
            sg = self._request("GET", "/api/tmc/sg", params={"axis": 2})
            if sg.get("ok"):
                st.sg_result = float(sg.get("sg_result") or 0)
                st.tstep = float(sg.get("tstep") or 0)
        except requests.RequestException:
            pass
        self.last_status = st
        return st

    def set_velocity(self, pan: float = 0.0, tilt: float = 0.0, zoom: float = 0.0) -> None:
        self.last_cmd = {"pan": float(pan), "tilt": float(tilt), "zoom": float(zoom)}
        self._request(
            "POST",
            "/api/velocity",
            json={"pan": float(pan), "tilt": float(tilt), "zoom": float(zoom)},
        )

    def stop(self) -> None:
        self.last_cmd = {"pan": 0.0, "tilt": 0.0, "zoom": 0.0}
        try:
            self._request("POST", "/api/command", json={"command": "stop"})
        except requests.RequestException:
            try:
                self.set_velocity(0, 0, 0)
            except requests.RequestException:
                pass

    def home(self) -> None:
        self._request("POST", "/api/command", json={"command": "home"})

    def set_preset_recall(self, enabled: bool) -> None:
        self._request("POST", "/api/preset/recall", json={"enabled": bool(enabled)})

    def mute_automations(self) -> None:
        st = self.positions()
        if st.ok and self._saved_recall is None:
            self._saved_recall = st.preset_recall
        self.set_preset_recall(False)

    def restore_automations(self) -> None:
        if self._saved_recall is None:
            return
        try:
            self.set_preset_recall(self._saved_recall)
        except requests.RequestException:
            pass
        self._saved_recall = None

    def presets(self) -> list[dict]:
        data = self._request("GET", "/api/presets")
        return list(data.get("presets") or [])

    def goto_preset(self, index: int) -> dict:
        return self._request("POST", "/api/preset/goto", json={"index": int(index)})

    def set_irun(self, **kwargs: int) -> dict:
        body = {k: int(v) for k, v in kwargs.items() if v is not None}
        return self._request("POST", "/api/tmc/irun", json=body)

    def get_irun(self) -> dict:
        return self._request("GET", "/api/tmc/irun")
