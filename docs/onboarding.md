# Onboarding

Goal: clone → running quickstart in under 10 minutes.

## 1. Install

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate    macOS/Linux: source .venv/bin/activate
pip install -e ".[dev]"
```

(`uv sync` works too if you use [uv](https://docs.astral.sh/uv/) — `pyproject.toml` doesn't lock you into a particular installer.)

## 2. Run the tests

```bash
pytest
```

This exercises the adaptor registry, both exporters, and the pipeline end to end — if it's green, your environment is set up correctly.

## 3. Run the quickstart

```bash
python examples/quickstart.py
```

Wraps a plain function in `LegacyFunctionAdaptor`, runs it through `Pipeline`, writes the result to `quickstart_output.json`. Read that file alongside [architecture.md](architecture.md) — it's the shortest path through every layer of the SDK.

## 4. Launch the desktop app

```bash
pip install -e ".[ui]"
research-sdk-ui
```

Opens a native Qt (PySide6) window with tabs for the adaptor registry, the quickstart pipeline, a Monte Carlo simulation demo, an offline 2D planner view (no grSim/Phoenix needed -- steps the vendored voronoi planner locally and draws it routing around obstacles), and grSim control (needs a running grSim instance to actually move a robot -- runs on a background thread so the window stays responsive while it drives). Source: `src/research_sdk/ui/app.py`.

**Run recording (planned, not yet implemented).** Each runnable tab will get a "record" checkbox next to its run button. Checking it before running an example logs a snapshot per step; the log is flushed to CSV (via `CSVExporter`, see [exporting.md](exporting.md)) either when the run finishes on its own or when the user stops it early -- so a paper-ready table comes out of the same click that runs the example, no separate export step. See [decisions/0004-layered-pipeline-and-run-recording.md](decisions/0004-layered-pipeline-and-run-recording.md).

## 5. Try the MCP server

```bash
research-sdk-mcp
```

Point Claude Code's MCP config (or any other MCP client) at this command to see `list_adaptors` show up as a callable tool. See [mcp.md](mcp.md) before adding your own tool — there's a reason tools are pinned instances, not a generic dispatcher.

## 6. Where to go next

- Adding a new data source → [adaptors.md](adaptors.md)
- Adding a new export destination → [exporting.md](exporting.md)
- Exposing something to an agent → [mcp.md](mcp.md)
- Why things are shaped this way → [architecture.md](architecture.md) and [decisions/](decisions/)

## If something's broken

Check `docs/decisions/` first — a surprising constraint (e.g. "why isn't there a generic MCP dispatcher") is usually explained there before it's explained in code comments.
