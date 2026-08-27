# Exporting

## The contract

```python
class Exporter(ABC):
    @abstractmethod
    def export(self, records: Iterable[dict[str, Any]], destination: Path) -> Path: ...
```

Every exporter takes `records` (an iterable of flat dicts) and a `destination` path, writes them, and returns the path actually written. Nothing upstream of an exporter needs to know which format it's writing.

## Built in

| Exporter | Format | Extra deps |
|---|---|---|
| `JSONExporter` | `.json`, pretty-printed | none |
| `CSVExporter` | `.csv`, header from first record's keys | none |
| `ParquetExporter` | `.parquet` | `pip install "research-sdk[parquet]"` (pyarrow) |

`CSVExporter` assumes every record has the same keys as the first one — if your adaptor can return heterogeneous records, normalize them before exporting rather than adding branching logic to the exporter.

**Planned consumer: UI run recording.** The Qt UI's planned "record" checkbox (see [onboarding.md](onboarding.md)) will write its per-step log through `CSVExporter`, on the same "one flat record per row" assumption above — so a recorded run lands as a table you can pull straight into a paper, with no reformatting step. Not implemented yet; tracked in [decisions/0004-layered-pipeline-and-run-recording.md](decisions/0004-layered-pipeline-and-run-recording.md).

## Adding a new destination

A cloud destination (S3, BigQuery, a database, Google Sheets) is a new `Exporter` subclass, same as a new local format — `export()` still returns a `Path`-like handle or identifier for where the data landed. Keep credentials/config in the exporter's constructor, same rule as adaptors (see [adaptors.md](adaptors.md)): never take them as arguments a caller passes at export time from an untrusted context.
