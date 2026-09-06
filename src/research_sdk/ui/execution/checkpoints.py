"""Append-only execution checkpoint records backed by events.jsonl."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class CheckpointRecord:
    run_id: str
    checkpoint_id: str
    parent_run_id: str | None
    scenario_name: str
    scenario_hash: str
    triggers: tuple[dict[str, Any], ...]
    robots: tuple[dict[str, Any], ...]
    ball: dict[str, Any] | None
    waypoint_indexes: dict[str, int]
    velocity_owner: str
    path_ids: dict[str, str]
    metrics: dict[str, Any]
    state: str
    timestamp_utc: str

    @classmethod
    def create(cls, **values) -> CheckpointRecord:
        return cls(timestamp_utc=datetime.now(UTC).isoformat(), **values)

    def to_event(self) -> dict[str, Any]:
        return {"event": "checkpoint_created", **asdict(self)}

    @classmethod
    def from_event(cls, event: dict[str, Any]) -> CheckpointRecord:
        values = dict(event)
        values.pop("event", None)
        values["triggers"] = tuple(values["triggers"])
        values["robots"] = tuple(values["robots"])
        return cls(**values)


class CheckpointStore:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, checkpoint: CheckpointRecord) -> None:
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(checkpoint.to_event(), separators=(",", ":")) + "\n")
            stream.flush()

    def list(self) -> tuple[CheckpointRecord, ...]:
        if not self.path.exists():
            return ()
        records = []
        for line in self.path.read_text(encoding="utf-8").splitlines():
            event = json.loads(line)
            if event.get("event") == "checkpoint_created":
                records.append(CheckpointRecord.from_event(event))
        return tuple(records)

    def get(self, checkpoint_id: str) -> CheckpointRecord:
        for checkpoint in self.list():
            if checkpoint.checkpoint_id == checkpoint_id:
                return checkpoint
        raise KeyError(checkpoint_id)
