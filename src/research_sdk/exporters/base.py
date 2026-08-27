from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Iterable


class Exporter(ABC):
    """Common interface for writing pipeline results to a destination."""

    @abstractmethod
    def export(self, records: Iterable[dict[str, Any]], destination: Path) -> Path:
        """Write records to destination and return the path written."""
