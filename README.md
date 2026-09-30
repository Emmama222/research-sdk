# research-sdk

This is a tool set that would provide the basics to perform further development and evaluation. 

## Project map

See [GRAPHIFY.md](GRAPHIFY.md) for visual maps of the system flow, package
dependencies, planner family, and execution lifecycle.

## Fast headless simulation

Run scenarios without Qt, UDP, or a grSim window and export raw plus aggregated
research results:

    research-sdk-headless scenarios/crowded.json --planner all --trials 10

With no input, every JSON file directly in `scenarios/` is run
(`crowded`, `dynamic2`, `snapshot_…` and the dated UI scenarios).

See [docs/headless.md](docs/headless.md) for the simulation model, metrics,
batch options, and Python API.

## Running the ACRA 2026 scenarios

The 200 frozen evaluation scenarios live in `scenarios/acra2026-200/`
(previews in `scenario_png/`). Run the three result sets from the repository
root; each one uses exactly those 200 scenarios and refuses to start otherwise:

```powershell
python -m research_sdk.headless --result-set one-shot-validation --output-dir results/acra2026-final/one-shot-validation
python -m research_sdk.headless --result-set policy-comparison --workers 1 --output-dir results/acra2026-final/policy-comparison
python -m research_sdk.headless --result-set horizon-sweep --workers 1 --output-dir results/acra2026-final/horizon-sweep
python scripts/build_result_tables.py
```

The one-shot set takes well under a minute. The two dynamic sets use one
worker so the wall-clock latency numbers are clean. The tables are written to
`results/acra2026-final/tables/`. Arms, metrics and failure rules are described
under "ACRA 2026 result sets" in [docs/headless.md](docs/headless.md);
`results/README.md` explains the results layout.

For native grSim/ODE physics validation, see [docs/physics.md](docs/physics.md).
After building the engine in WSL, run from PowerShell:

```powershell
.\scripts\physics.ps1 scenarios/crowded.json --planner all
```

Current Project available : 
- Path Planning System Comparison (2026) 


## What does `research-sdk` provides ? 

This software provides : 

- offline sandbox simulation (point to point 2D Simulation)

< Pictures will be added soon >

- Connection to grSim Simulation 

< Pictures will be added soon >

- Connection to TurtleRabbit - Phenoix server
< Pictures will be added soon >

(Future adaptation)


- World model and map produced from What was received over the network from Small Size League Vision System. 

- basic robot control function wrapper. 

## Installation and dependencies
This software is designed as a local python software. Please run the following to install. Using a virtual environment is highly recommended.

### Installing and activate Virtual Environment
```shell
python3 -m venv .venv 
source .venv/bin/activate # Linux or MacOS
source .venv/Scripts/activate # Windows
```

```shell
pip install -e .[dev]
```

