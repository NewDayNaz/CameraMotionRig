from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


# Mirror of camera_fysetc_e4/main/stepper_limits.h so suggestions can compare.
FIRMWARE = {
    "MIN_PAN_TILT_VELOCITY": 20.0,
    "MIN_ZOOM_VELOCITY": 70.0,
    "MAX_PAN_VELOCITY": 722.5,
    "MAX_TILT_VELOCITY": 1020.0,
    "MAX_ZOOM_VELOCITY": 180.0,
    "MAX_PAN_RANGE_STEPS": 18400,
    "MAX_TILT_RANGE_STEPS": 15230,
    "MAX_ZOOM_RANGE_STEPS": 4000,
    "HOMING_PAN_VELOCITY": 170.0,
    "HOMING_TILT_VELOCITY": 450.0,
    "HOMING_ZOOM_VELOCITY": 50.0,
    "HOMING_PAN_SLOW_VELOCITY": 34.0,
    "HOMING_TILT_SLOW_VELOCITY": 85.0,
    "HOMING_ZOOM_SLOW_VELOCITY": 25.0,
    "PRESET_PAN_TILT_VELOCITY": 127.5,
    "PRESET_ZOOM_VELOCITY": 80.0,
    "PRESET_RAMP_S": 0.4,
    "HOME_TILT_PULLOFF_STEPS": 80,
    "ZOOM_PT_SCALE_MIN": 0.5,
}

TRAVEL_SIGN = {"pan": -1.0, "tilt": -1.0, "zoom": 1.0}
AXIS_MAX_VEL = {
    "pan": FIRMWARE["MAX_PAN_VELOCITY"],
    "tilt": FIRMWARE["MAX_TILT_VELOCITY"],
    "zoom": FIRMWARE["MAX_ZOOM_VELOCITY"],
}
AXIS_MAX_RANGE = {
    "pan": FIRMWARE["MAX_PAN_RANGE_STEPS"],
    "tilt": FIRMWARE["MAX_TILT_RANGE_STEPS"],
    "zoom": FIRMWARE["MAX_ZOOM_RANGE_STEPS"],
}

SWEEP_SPEEDS = {
    "pan": [40, 80, 120, 180, 250, 350, 450, 550, 650, 722],
    "tilt": [40, 80, 120, 180, 250, 350, 450, 600, 750, 900, 1020],
    "zoom": [70, 80, 100, 120, 140, 160, 180],
}

# Cable-safe window from the pose at trial start. Pan travel is left (−);
# a full sweep+range used to walk ~180°. Stay well inside that.
TUNER_TRAVEL_BUDGET = {"pan": 700, "tilt": 900, "zoom": 350}
SWEEP_MEASURE_S = {"pan": 0.70, "tilt": 0.70, "zoom": 1.15}
SWEEP_SPINUP_S = {"pan": 0.30, "tilt": 0.30, "zoom": 0.45}


@dataclass
class TunerSettings:
    controller_url: str = "http://192.168.1.50"
    ndi_source: str = ""
    video_source: str = "ndi"  # ndi | webcam | demo
    webcam_index: int = 0

    @staticmethod
    def path() -> Path:
        root = Path(os_appdata()) / "CameraMotionRig"
        root.mkdir(parents=True, exist_ok=True)
        return root / "vision_tuner.json"

    @classmethod
    def load(cls) -> "TunerSettings":
        p = cls.path()
        if not p.is_file():
            return cls()
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})
        except (OSError, json.JSONDecodeError, TypeError):
            return cls()

    def save(self) -> None:
        self.path().write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")


def os_appdata() -> Path:
    import os

    appdata = os.environ.get("APPDATA")
    if appdata:
        return Path(appdata)
    return Path.home() / "AppData" / "Roaming"


def runs_dir() -> Path:
    p = Path(__file__).resolve().parent.parent / "runs"
    p.mkdir(parents=True, exist_ok=True)
    return p
