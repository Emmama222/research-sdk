"""Start/stop the built-in simulator as a child process (used by the Qt console).

A separate process keeps the 240 Hz physics loop off the UI's GIL and lets a
crash in either side leave the other running.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Sequence
from pathlib import Path

from research_sdk.sim.server import READY_MARKER

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]  # .../src


class SimulatorProcess:
    def __init__(self, python: str = sys.executable) -> None:
        self.python = python
        self.process: subprocess.Popen[str] | None = None
        self.output: deque[str] = deque(maxlen=200)
        self._ready = threading.Event()
        self._reader: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def command(self, args: Sequence[str] = ()) -> list[str]:
        return [self.python, "-u", "-m", "research_sdk.sim", "--stop-on-stdin-eof", *args]

    def start(self, args: Sequence[str] = (), *, ready_timeout_s: float = 5.0) -> None:
        """Launch and wait for the ready line; raise with the log tail on failure."""
        if self.running:
            return
        self.output.clear()
        self._ready.clear()
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(
            [str(_PACKAGE_ROOT), *filter(None, [env.get("PYTHONPATH")])]
        )
        kwargs = {}
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
        self.process = subprocess.Popen(
            self.command(args),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.PIPE,  # closing it (or our death) stops the child
            text=True,
            env=env,
            **kwargs,
        )
        self._reader = threading.Thread(target=self._read, args=(self.process,), daemon=True)
        self._reader.start()
        deadline = time.monotonic() + ready_timeout_s
        while time.monotonic() < deadline:
            if self._ready.wait(0.05):
                return
            if self.process.poll() is not None:
                break
        tail = "\n".join(self.output) or "(no output)"
        self.stop()
        raise RuntimeError(f"Built-in simulator did not start:\n{tail}")

    def _read(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            line = line.rstrip()
            self.output.append(line)
            if line.startswith(READY_MARKER):
                self._ready.set()

    def stop(self, timeout_s: float = 3.0) -> int | None:
        process = self.process
        if process is None:
            return None
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        if process.poll() is None:
            try:
                process.wait(1.0)  # graceful: stdin EOF stops the loop
            except subprocess.TimeoutExpired:
                process.terminate()
            try:
                process.wait(timeout_s)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout_s)
        if self._reader is not None:
            self._reader.join(1.0)
        self.process = None
        return process.returncode
