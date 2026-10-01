"""Describe the machine a batch runs on, for result manifests.

The 100 ms replan limit (DEC-019) is wall-clock, so how many robots it stops
depends on the CPU. Every manifest records this block so a batch can be tied to
the machine that produced it. Run ``python -m research_sdk.machine_info`` to
print it on its own.

No hostname, user name or serial numbers are recorded.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import sys
from importlib import metadata

_PACKAGES = ("numpy", "scipy", "networkx", "shapely", "scikit-learn")


def _cpu_model() -> str:
    system = platform.system()
    try:
        if system == "Windows":
            import winreg  # type: ignore[import-not-found]

            key = winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0"
            )
            return str(winreg.QueryValueEx(key, "ProcessorNameString")[0]).strip()
        if system == "Darwin":
            out = subprocess.run(
                ["sysctl", "-n", "machdep.cpu.brand_string"],
                capture_output=True, text=True, timeout=5, check=False,
            )
            if out.stdout.strip():
                return out.stdout.strip()
        if system == "Linux":
            with open("/proc/cpuinfo", encoding="utf-8") as handle:
                for line in handle:
                    if line.lower().startswith("model name"):
                        return line.split(":", 1)[1].strip()
    except Exception:  # noqa: BLE001 -- best effort, never break a batch
        pass
    return platform.processor() or "unknown"


def _memory_gb() -> float | None:
    try:
        import psutil  # type: ignore[import-not-found]

        return round(psutil.virtual_memory().total / 1024**3, 1)
    except Exception:  # noqa: BLE001
        pass
    try:
        if platform.system() == "Windows":
            import ctypes

            class _MemoryStatus(ctypes.Structure):
                _fields_ = [
                    ("dwLength", ctypes.c_ulong),
                    ("dwMemoryLoad", ctypes.c_ulong),
                    ("ullTotalPhys", ctypes.c_ulonglong),
                    ("ullAvailPhys", ctypes.c_ulonglong),
                    ("ullTotalPageFile", ctypes.c_ulonglong),
                    ("ullAvailPageFile", ctypes.c_ulonglong),
                    ("ullTotalVirtual", ctypes.c_ulonglong),
                    ("ullAvailVirtual", ctypes.c_ulonglong),
                    ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
                ]

            status = _MemoryStatus()
            status.dwLength = ctypes.sizeof(_MemoryStatus)
            if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):  # type: ignore[attr-defined]
                return round(status.ullTotalPhys / 1024**3, 1)
        if platform.system() == "Linux":
            with open("/proc/meminfo", encoding="utf-8") as handle:
                kib = int(handle.readline().split()[1])
            return round(kib / 1024**2, 1)
    except Exception:  # noqa: BLE001
        pass
    return None


def _physical_cores() -> int | None:
    try:
        import psutil  # type: ignore[import-not-found]

        return psutil.cpu_count(logical=False)
    except Exception:  # noqa: BLE001
        return None


def _versions() -> dict[str, str | None]:
    versions: dict[str, str | None] = {}
    for name in _PACKAGES:
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def machine_info() -> dict:
    """Return a JSON-serialisable description of this machine and interpreter."""
    return {
        "os": platform.platform(),
        "os_system": platform.system(),
        "machine": platform.machine(),
        "cpu_model": _cpu_model(),
        "cpu_logical_cores": os.cpu_count(),
        "cpu_physical_cores": _physical_cores(),
        "memory_gb": _memory_gb(),
        "python": sys.version.split()[0],
        "python_implementation": platform.python_implementation(),
        "packages": _versions(),
    }


if __name__ == "__main__":
    print(json.dumps(machine_info(), indent=2))
