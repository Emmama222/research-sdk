from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

from .base import Exporter


class JSONExporter(Exporter):
    def export(self, records: Iterable[dict[str, Any]], destination: Path) -> Path:
        destination.write_text(json.dumps(list(records), indent=2))
        return destination
