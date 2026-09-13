# results/ index

Five markdown files here. Each `section_*` file matches a section in the paper (or a trigger
condition referenced from one); `reference_*` is a cross-cutting doc that ties the section
files together. Raw per-trial data (JSON + matching `.log`) sits alongside these, named
`grsim_<trigger>_<planner>_<scenario>_n<samples>`.

| File | Answers | Offline comparison? |
|---|---|---|
| `section_4.1_static_environment.md` | §4.1: three backends, four stationary-obstacle scenarios, live vs. offline | Yes -- matches almost exactly |
| `section_4.2_no_trigger.md` | §4.2: three backends with no replanning trigger at all (replan every tick) | Yes -- and it's the sharpest contradiction in the project: offline predicts Voronoi almost totally fails (0.5-8.5% arrival), live it arrives 100% of the time |
| `section_4.3_event_trigger.md` | §4.3: three backends under the production trigger (`evaluate_route`, from `backbackup`) | None -- production code, never run offline |
| `section_geometric_trigger.md` | Three backends under da Silva Costa & Tonidandel's geometric trigger, live only *and* live vs. offline. Referenced from both 4.2 and 4.3 as the third comparison point; not itself numbered 4.2/4.3 in the paper. | Yes -- ranking holds, absolute costs (replans, nav time) are substantially higher live |
| `reference_all_triggers_compared.md` | All three trigger conditions (no-trigger / production / geometric) side by side across every backend and scenario, plus the full PRM seed-bug diagnosis | N/A -- this is the live-only deep dive the three section files draw their cross-trigger claims from |

## The one thing to know before reading any of these

A bug in `drive_grsim.py`'s PRM wrapper (it never varied PRM's random seed, so every replan
rebuilt one fixed roadmap) caused a false 0/60 "PRM completely fails" result on the first pass
through the no-trigger data. It was found, fixed, and every affected batch was re-run. All
numbers in all five files above are the corrected, post-fix results. Full diagnosis is in
`reference_all_triggers_compared.md`'s setup notes.

## The headline result

Offline's own stability study predicts Voronoi under 50ms-periodic (no-gating) replanning
almost completely fails -- 8.5% and 0.5% arrival, ~320-360 reversals/run, an order of magnitude
past its own "always fails" cutoff of 14.7. Live, under the mechanically identical condition,
Voronoi arrives 100% of the time with reversals two orders of magnitude lower. This is the one
place live and offline don't just disagree on magnitude -- they disagree on the conclusion.
Full comparison in `section_4.2_no_trigger.md`.

## Static vs. dynamic, the short version

- **Static (§4.1): live confirms offline almost exactly.** No execution, no timing, no
  replanning dynamics -- nothing for live and offline to disagree on.
- **Dynamic (§4.2, §4.3, geometric): live and offline agree on which backend is better, but
  disagree substantially on the absolute numbers.** Replans and nav time are consistently
  higher live across every condition tested -- real execution dynamics the offline model
  doesn't capture.
