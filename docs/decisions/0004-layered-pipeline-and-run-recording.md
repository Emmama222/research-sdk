# 0004: Three-layer pipeline model, and UI-triggered run recording

Date: 2026-08-20

## Context

The original architecture doc (see ADR 0001) described a flat "Backend /
Front end" split. That undersells how data actually needs to move: input
(vision, game controller, robot feedback) needs to populate a world model
before anything else can happen; a middle stage turns that world model into
a per-robot decision; and a separate stage has to convert that decision into
whatever wire format the destination (grSim, another simulator, eventually
real robots) expects and send it. Keeping "decide" and "format + send"
conflated made it harder to reason about what a new destination (e.g. real
robots) would actually require.

Separately: the UI currently has no way to produce data for the research
paper other than a manual export step after the fact. The ask is to let
recording happen inline with running an example — check a box, run the
example, get a CSV.

## Decisions

1. **Adopt a three-layer model: input/world-model → middle
   (map/processing/UI) → output (format/send).** Documented in
   [../architecture.md](../architecture.md). The UI is explicitly part of
   the middle layer, not a separate front-end tier — it observes and
   triggers processing, it doesn't own a different concern from the rest of
   that layer.

2. **This is a target architecture, not a description of today's code.**
   Notably, `GrSimAdaptor.fetch()` currently computes its control step *and*
   sends the resulting packet in one call — decide and send aren't split
   yet. `architecture.md` says so explicitly rather than papering over the
   gap. No code changes as part of this ADR; that's deliberate, follow-up
   work.

3. **Run recording is UI-triggered, not always-on.** A checkbox next to each
   runnable tab's run button opts a given run into logging. This keeps the
   default (fast iteration, no log clutter) separate from the deliberate
   "I want this run for the paper" case.

4. **The log flushes on run end, however the run ends.** Both a run
   finishing on its own and the user stopping it early produce a flush —
   partial data from an aborted run is still useful for debugging or a
   "this approach didn't converge" note, and requiring a clean finish would
   silently drop that.

5. **Output format is CSV, via the existing `CSVExporter`.** No new export
   format needed — recording reuses the exporter contract in
   [../exporting.md](../exporting.md) (flat `dict` records, header from the
   first record's keys). A recorded run's per-step snapshots are exactly the
   kind of homogeneous-record data that contract already assumes, and CSV
   is what drops directly into a paper's table pipeline.

## Status

Accepted, implementation pending (next session). Revisit if: the
input/decide/output split needs to become real in code (e.g. when real
robots make "decide" vs. "send" a safety-relevant boundary, not just a
documentation one), or if recording needs a format beyond flat CSV rows
(e.g. nested per-robot data that doesn't flatten cleanly).
