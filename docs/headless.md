# Headless simulation

For validation with the actual grSim/ODE physics engine, use
[`--backend grsim`](physics.md). The fast kinematic backend described below
remains the default.

The headless runner executes saved scenarios on a deterministic virtual clock.
It loads the same scenario files and planner implementations as the Qt
application, but it does not open Qt, send UDP packets, start grSim, or sleep
between control ticks.

This makes planner experiments much faster than real time and suitable for
repeatable local batches or CI.

## Quick start

Install the project, then run every saved scenario against every planner:

~~~shell
pip install -e .
research-sdk-headless
~~~

Run one scenario and repeat each selected planner 20 times:

~~~shell
research-sdk-headless scenarios/crowded.json +  --planner visibility prm voronoi +  --trials 20 +  --seed 100 +  --output-dir results/crowded_batch
~~~

Python module invocation is equivalent:

~~~shell
python -m research_sdk.headless scenarios/crowded.json --planner all
~~~

Directories are accepted as inputs and every JSON file directly inside them is
run. With no input, the command uses the *scenarios/* directory.

## Simulation model

The runner mirrors the current application at the behavioral boundary:

1. Build planner obstacles from the selected scenario and the other robots.
2. Plan once per robot with the selected production planner adapter.
3. Execute all robot waypoint paths concurrently.
4. Apply the same proportional controller gain, configured maximum speed,
   120 mm intermediate tolerance, and 60 mm final tolerance.
5. Advance moving scenario obstacles from their configured linear velocity.
6. Count robot-to-robot and robot-to-obstacle collision episodes.

The virtual clock advances by *--dt-ms* without sleeping. The reported
real-time factor is simulated duration divided by wall-clock runtime.

This is a point-kinematic research backend, not grSim rigid-body physics. It
does not model wheel dynamics, acceleration, friction, ball contact, radio/UDP
latency, or vision noise. Use it for fast algorithm screening and regression
batches; use real grSim for final physics validation. Every result manifest
records this distinction with *physics_equivalence* set to false.

## Result files

Each invocation creates:

| File | Contents |
|---|---|
| **runs.csv** | One flat row per scenario, planner, and trial |
| **runs.json** | The same raw records as JSON |
| **summary.csv** | Aggregates grouped by scenario and planner |
| **summary.json** | The same aggregates as JSON |
| **manifest.json** | Engine model, configuration, planners, scenarios, and seeds |

Raw result columns include:

- completion status, planning failures, and failed robot IDs;
- total and mean planning latency;
- simulated duration, wall duration, and real-time factor;
- robot and static-obstacle collision episodes;
- planned and travelled path length;
- mean and maximum final target error;
- minimum clearance, where a negative value is penetration depth; and
- tick count and virtual timestep.

The summary reports completion rate, median and p95 planning latency, total
collision episodes, mean simulated and wall duration, mean real-time factor,
path lengths, final error, and worst clearance.

## Useful options

~~~text
--planner all|voronoi|prm|visibility
--trials N
--seed N
--dt-ms MILLISECONDS
--max-sim-seconds SECONDS
--speed-mps METRES_PER_SECOND
--gain PER_SECOND
--output-dir PATH
--fail-on-incomplete
~~~

The *--fail-on-incomplete* option returns exit status 2 when any trial times
out or has a planning failure, which is useful in automated regression jobs.

## Python API

~~~python
from research_sdk.headless import SimulationConfig, run_experiments, write_results
from research_sdk.ui.scenarios import ScenarioStore

scenario = ScenarioStore().load('crowded')
config = SimulationConfig(dt_s=0.01, max_simulation_s=20.0)
results = run_experiments(
    [scenario],
    ['visibility', 'prm'],
    trials=10,
    seed=42,
    config=config,
)
write_results(results, 'results/headless_example', config=config)
~~~
