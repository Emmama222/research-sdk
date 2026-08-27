from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

from .base import Exporter


class ParquetExporter(Exporter):
    """Requires the optional `pyarrow` extra: pip install "research-sdk[parquet]"."""

    def export(self, records: Iterable[dict[str, Any]], destination: Path) -> Path:
        import pyarrow as pa
        import pyarrow.parquet as pq

        table = pa.Table.from_pylist(list(records))
        pq.write_table(table, destination)
        return destination
