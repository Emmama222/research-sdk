# 0001: Initial architecture

Date: 2026-08-01

## Context

The SDK started as a bare README ("evaluation and quick deployment with old code") with no code. It needed: a way to wrap both legacy internal code and external APIs uniformly, a way to export results, and a way to expose capabilities to agents via MCP.

## Decisions

1. **One `Adaptor` interface for both legacy code and external APIs.** A single `fetch(**kwargs) -> Any` contract, implemented by `LegacyFunctionAdaptor` (wraps a Python callable) and `APIAdaptor` (wraps a JSON HTTP API via a shared retrying client). Rationale: evaluation/export code shouldn't need to know or care which kind of source it's reading from.

2. **MCP server, not MCP client.** The SDK exposes its own tools over MCP so agents can drive it directly. It does not consume external MCP servers as adaptors — no current use case needs that, and adding an MCP client is a real addition, not a small one. Revisit with a new ADR if that need appears.

3. **MCP tools are pinned instances, not a generic `run_adaptor(name, params)` dispatcher.** A generic dispatcher would let a calling agent control adaptor construction arguments (e.g. an API's base URL), which is effectively an open proxy, and produces useless tool schemas. See [../mcp.md](../mcp.md).

4. **Registry is opt-in, gated on "constructible with no runtime-only arguments."** Adaptors needing a live object (a function reference, an open connection) are wired up in code, not looked up by name. This is the same bar used for "is this adaptor MCP-safe."

5. **Exporters take `list[dict]` and a `Path`, nothing fancier.** New destinations (S3, BigQuery, ...) are new `Exporter` subclasses; the pipeline and adaptors never change to support them.

## Status

Accepted. Revisit if: an MCP client becomes necessary, the pipeline needs branching/retry logic beyond "run one adaptor, write one exporter," or export destinations need shared plumbing (auth, batching) the way API adaptors share `RetryingClient` — at that point a `clients`-style layer for exporters is worth its own ADR.
