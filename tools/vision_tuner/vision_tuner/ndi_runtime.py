from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

_DLL_NAME = "Processing.NDI.Lib.x64.dll"
_READY = False
_INFO: Optional["NdiRuntimeInfo"] = None


@dataclass
class NdiRuntimeInfo:
    found: bool
    path: str = ""
    version: str = ""
    error: str = ""

    def summary(self) -> str:
        if self.found:
            ver = self.version or "NDI Runtime 6"
            return f"{ver}  ({self.path})"
        return self.error or "NDI Runtime 6 not found"


def _candidate_dirs() -> list[Path]:
    dirs: list[Path] = []
    env = os.environ.get("NDI_RUNTIME_DIR") or os.environ.get("NDI_SDK_DIR")
    if env:
        p = Path(env)
        dirs.append(p)
        dirs.append(p / "Bin" / "x64")
        dirs.append(p / "v6")
    pf = os.environ.get("ProgramFiles", r"C:\Program Files")
    pfx86 = os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")
    dirs.extend(
        [
            Path(pf) / "NDI" / "NDI 6 Runtime" / "v6",
            Path(pfx86) / "NDI" / "NDI 6 Runtime" / "v6",
            Path(pf) / "NDI" / "NDI 6 SDK" / "Bin" / "x64",
            Path(pf) / "NDI" / "NDI 5 Runtime" / "v5",
        ]
    )
    return dirs


def find_runtime_dll() -> Optional[Path]:
    for d in _candidate_dirs():
        dll = d / _DLL_NAME
        if dll.is_file():
            return dll
    return None


def _read_version(dll_path: Path) -> str:
    try:
        import ctypes

        lib = ctypes.WinDLL(str(dll_path))
        fn = lib.NDIlib_version
        fn.restype = ctypes.c_char_p
        raw = fn()
        if not raw:
            return "NDI Runtime 6"
        text = raw.decode("utf-8", errors="replace").strip()
        return text.split("\n", 1)[0]
    except Exception:
        return "NDI Runtime 6"


def ensure_ndi_runtime() -> NdiRuntimeInfo:
    """Put NDI Runtime 6 on the DLL search path before cyndilib/NDIlib load."""
    global _READY, _INFO
    if _INFO is not None:
        return _INFO

    dll = find_runtime_dll()
    if dll is None:
        _INFO = NdiRuntimeInfo(
            found=False,
            error=(
                "NDI Runtime 6 DLL not found. Expected "
                r"C:\Program Files\NDI\NDI 6 Runtime\v6\Processing.NDI.Lib.x64.dll"
            ),
        )
        return _INFO

    folder = str(dll.parent)
    path_parts = os.environ.get("PATH", "").split(os.pathsep)
    if folder not in path_parts:
        os.environ["PATH"] = folder + os.pathsep + os.environ.get("PATH", "")
    if sys.platform == "win32":
        try:
            os.add_dll_directory(folder)
        except (OSError, AttributeError):
            pass

    _READY = True
    _INFO = NdiRuntimeInfo(found=True, path=folder, version=_read_version(dll))
    return _INFO
