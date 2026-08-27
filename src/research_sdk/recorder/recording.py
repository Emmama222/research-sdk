"""Snapshot recording and replay helpers.

JSON Lines is used as the replay format: one serialized WorldSnapshot per
line. This keeps replay append-friendly and avoids flattening nested robot
state into a wide CSV schema.
"""

from __future__ import annotations

import json
import queue
import threading
from pathlib import Path

from research_sdk.world.snapshot import (
    WorldSnapshot,
    snapshot_from_dict,
    snapshot_to_dict,
)


class SnapshotRecorder:
    """Append world snapshots to a replay folder."""

    def __init__(
        self,
        folder: str | Path,
        filename: str = "snapshots.jsonl",
        flush_each: bool = False,
    ):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder / filename
        self.flush_each = flush_each
        self._file = self.path.open("a", encoding="utf-8")

    def write(self, snapshot: WorldSnapshot) -> None:
        json.dump(snapshot_to_dict(snapshot), self._file, separators=(",", ":"))
        self._file.write("\n")
        if self.flush_each:
            self._file.flush()

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        self.flush()
        self._file.close()

    def __enter__(self) -> "SnapshotRecorder":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


class AsyncSnapshotRecorder:
    """Queue-backed snapshot recorder for live control loops.

    `write()` never performs disk IO. It enqueues the snapshot and returns
    True if accepted, False if the queue is full. Dropping is preferable to
    blocking the control/BT loop.
    """

    def __init__(
        self,
        folder: str | Path,
        filename: str = "snapshots.jsonl",
        max_queue: int = 512,
    ):
        self._queue: queue.Queue[WorldSnapshot | None] = queue.Queue(
            maxsize=max_queue
        )
        self._recorder = SnapshotRecorder(folder, filename)
        self._dropped = 0
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name="SnapshotRecorder",
            daemon=True,
        )
        self._thread.start()

    @property
    def dropped(self) -> int:
        return self._dropped

    def write(self, snapshot: WorldSnapshot) -> bool:
        if self._closed:
            return False
        try:
            self._queue.put_nowait(snapshot)
            return True
        except queue.Full:
            self._dropped += 1
            return False

    def close(self, timeout: float | None = 2.0) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            # Make room for the sentinel; old replay data is less important
            # than a clean shutdown.
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self._queue.put_nowait(None)
        self._thread.join(timeout=timeout)
        if self._thread.is_alive():
            return
        self._recorder.close()

    def _run(self) -> None:
        while True:
            snapshot = self._queue.get()
            if snapshot is None:
                break
            self._recorder.write(snapshot)
        self._recorder.flush()

    def __enter__(self) -> "AsyncSnapshotRecorder":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


class SnapshotReplay:
    """Iterate snapshots from a replay folder or jsonl file."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        if self.path.is_dir():
            self.path = self.path / "snapshots.jsonl"

    def __iter__(self):
        with self.path.open("r", encoding="utf-8") as file:
            for line in file:
                line = line.strip()
                if not line:
                    continue
                yield snapshot_from_dict(json.loads(line))
