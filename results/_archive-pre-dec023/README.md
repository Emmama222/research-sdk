# Results with the shared escape step, before the Voronoi check margin (archived 1 Oct 2026)

`acra2026-final/` here is the complete ACRA 2026 set run on 1 Oct 2026 after
DEC-022 (shared escape step for all planners; commits `75ad6c2` / `62bc9e6`),
with tables and figures.

It was superseded by DEC-023: Voronoi's per-tick checks (event gate, escape
zone, direct-line and previous-route checks) ran at 0 mm extra margin, while PRM
and the visibility graph ran them at their 30 mm planning clearance. From DEC-023
Voronoi uses `voronoi_check_margin_mm: 30` for these checks (its roadmap is
unchanged). In an A/B test on all 200 scenarios (Linux workspace), Voronoi's
contact-free finishes rose from 103 to 135 (event @ 20 ms, no prediction) and
from 112 to 147 (cycle @ 20 ms), and were unchanged with a 100 ms horizon
(150 vs 146).

PRM and visibility-graph behaviour is identical to the new set; this folder is
the "Voronoi without check margin" reference. Do not use it for final claims.
