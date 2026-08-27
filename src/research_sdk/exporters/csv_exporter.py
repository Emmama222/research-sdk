from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Iterable

from .base import Exporter


class CSVExporter(Exporter):
    def export(self, records: Iterable[dict[str, Any]], destination: Path) -> Path:
        rows = list(records)
        if not rows:
            destination.write_text("")
            return destination
        with destination.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        return destination
