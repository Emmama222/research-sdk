# Native grSim physics validation

Use `--backend grsim` to run the actual grSim simulator with its Open Dynamics
Engine (ODE). Every trial starts a private native process using the official
`--headless` mode. Motors, chassis, wheels, ball, friction and physical contact
are simulated by grSim. The Python runner sends velocity commands and measures
the resulting motion from all four SSL-Vision cameras.

A private Xvfb display supplies the OpenGL context required by grSim; no
visible desktop window is opened. Software rendering is enabled for servers
and WSL machines without a dedicated GPU.

## Build once (Ubuntu or WSL)

In Ubuntu/WSL, install the upstream build dependencies:

```sh
sudo apt-get install build-essential cmake pkg-config qtbase5-dev libqt5opengl5-dev libgl1-mesa-dev libglu1-mesa-dev libprotobuf-dev protobuf-compiler libode-dev libboost-dev python3-venv xvfb xauth
bash scripts/build_grsim.sh
```

The build stays in the repository's ignored `.local/` directory. It pins grSim
to commit `fe2bd2915a46f9f11ea6cb48dc426b8047952073` and VarTypes to
`5853b6df05f801e8f2b9684b57731d056b383b86`. The included patch adds an isolated
configuration-file environment variable, resolves the installed robot models,
and pins the dependency. It does not
modify grSim's physics implementation. Each run records the actual source diff,
binary hash, ODE version, robot-model hash and configuration hash.

## Native Windows build (no WSL, no Xvfb)

grSim's source has no Linux-only code; what kept the backend in WSL was the
toolchain. MSYS2 (`C:\msys64`, installs without admin rights) provides gcc,
CMake, Ninja, Qt 5.15, ODE 0.16.6 and protobuf as prebuilt packages, so the
build takes minutes. From an MSYS2 MINGW64 shell, with the source cloned to a
path without spaces (`%USERPROFILE%\grsim\src`, not `%LOCALAPPDATA%`: a
packaged app such as the Claude desktop app has `AppData\Local` redirected
to a private cache, and a build made there is invisible to an ordinary shell)
and `scripts/grsim-isolated-config.patch` applied:

```bash
pacman -S --needed git mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake \
    mingw-w64-x86_64-ninja mingw-w64-x86_64-pkgconf mingw-w64-x86_64-qt5-base \
    mingw-w64-x86_64-ode mingw-w64-x86_64-protobuf
export CMAKE_POLICY_VERSION_MINIMUM=3.5   # VarTypes and the bundled protobuf 3.6.1 declare pre-3.5 minimums
cmake -S src -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_CLIENTS=OFF -DCMAKE_INSTALL_PREFIX=install
cmake --build build --parallel 8 && cmake --install build
```

Then stamp the install so the runner accepts it (it refuses a binary whose
hash is not in `build.json`):

```powershell
python scripts\record_grsim_build.py --prefix $env:USERPROFILE\grsim\install --source $env:USERPROFILE\grsim\src
```

On Windows the runner starts `grSim.exe --headless` directly: Qt's windows
platform plugin needs no virtual display, and the executable's Qt, ODE and
MinGW runtime DLLs are found by putting `C:\msys64\mingw64\bin` on the child
process's PATH (`RESEARCH_GRSIM_DLL_DIR` overrides that directory). Point the
runner at the executable with `--grsim-bin` or `RESEARCH_GRSIM_BIN`:

```powershell
$env:RESEARCH_GRSIM_BIN = "$env:USERPROFILE\grsim\install\bin\grSim.exe"
python -m research_sdk.headless scenarios/crowded.json --backend grsim --planner all --trials 3 --output-dir results/physics-crowded
$env:RESEARCH_RUN_PHYSICS = "1"; python -m pytest tests/test_grsim_physics.py -q
```

Verified 19 September 2026 on Windows 10 (build 19045), grSim fe2bd29 with
the patch, ODE 0.16.6: the three native tests (closed loop with evidence,
motor response and body collision, ball friction) pass in 10 s. The same
`--headless` mode is used on both platforms; with a window open on this
machine's integrated graphics the physics loop ran at 0.4 times real time,
so never run experiments with the GUI.

## Run experiments

From PowerShell, after building in WSL:

```powershell
.\scripts\physics.ps1 scenarios/crowded.json --planner all --trials 3 --output-dir results/physics-crowded
```

From Linux/WSL:

```sh
.local/physics-venv/bin/python -m research_sdk.headless scenarios/crowded.json --backend grsim --planner all --trials 3 --output-dir results/physics-crowded
```

Use a new output directory for each invocation. `--max-sim-seconds` controls
the timeout measured on grSim's clock. `--dt-ms` controls ODE's fixed timestep
(default 1000/120 ms, maximum 20 ms). Controller commands are updated from each
complete camera frame. Actual simulation speed depends on physics and host
performance; the kinematic backend's speedup figures do not apply here.

The current profile uses the Parsian robot model with 90 mm radius, the SDK's
9 m by 6 m field, zero artificial camera noise, and zero configured network
delay. Scenario obstacles are physical robots held at zero commanded velocity
or driven with their configured velocity. They can be pushed during contact.
Other obstacle radii, duplicate robot identities and out-of-field starts or
targets are rejected. Planner-specific obstacle layouts are respected.

## Validation and evidence

`completed` means all controlled robots reached their target tolerance and
stayed below 50 mm/s for at least 250 ms of simulation time.
`physics_validation_passed` additionally requires:

- no planning or engine errors;
- no missing execution frames or robot observations;
- consistent camera timestamps and fixed simulation steps; and
- zero observed robot/robot or robot/obstacle contact episodes.

The command exits with status 2 if any native trial does not meet these
criteria. The raw CSV reports the specific failure, completion, timeout,
missing-frame count, final speed and clearance. Summaries keep physics and
kinematic results in different groups.

Each run has an evidence directory containing:

- `scenario.json` and `paths.json`: exact experiment inputs and planner output;
- `trajectory.jsonl`: every observed simulation frame, robot poses, ball position,
  velocity commands calculated for the next frame, and sampled contacts;
- `grsim.xml` and `Parsian.ini`: simulator settings and robot model;
- `provenance.json`: engine, source, dependency and settings fingerprints;
- `result.json`: the final measurements and validation outcome; and
- `grsim.log`: native simulator output for diagnosing failures.

Robot contact metrics use circular proximity at each camera frame, with a 2 mm
tolerance for ODE contact separation. They are conservative sampled estimates,
not ODE's internal contact-force events. Contacts involving the ball are modeled
by the engine and ball positions are recorded, but are excluded from the path
planning contact pass criterion. Study small clearances with a smaller timestep
and the trajectory evidence. PRM seeds are recorded; bit-for-bit ODE replay
across operating systems or solver builds is not promised.

This supports final validation **within the recorded grSim model**. For claims
about a particular physical robot, calibrate its mass, wheel friction, motor
torque and dimensions against that hardware and confirm results on the robot.

## Check the real engine

See the [local verification report](physics-validation-results.md) for the tested
environment, sample planner results, and broader regression-suite limitations.

```sh
RESEARCH_RUN_PHYSICS=1 .local/physics-venv/bin/python -m pytest tests/test_grsim_physics.py -q
```

The native tests exercise target arrival and stopping, acceleration from rest,
body-to-body contact, and rolling-ball deceleration. Without the environment
flag, those native integration tests are explicitly skipped and only the
packet/data-boundary tests run.

Upstream references: [headless entry point](https://github.com/RoboCup-SSL/grSim/blob/fe2bd2915a46f9f11ea6cb48dc426b8047952073/src/main.cpp),
[ODE world integration](https://github.com/RoboCup-SSL/grSim/blob/fe2bd2915a46f9f11ea6cb48dc426b8047952073/src/sslworld.cpp),
and [installation guide](https://github.com/RoboCup-SSL/grSim/blob/fe2bd2915a46f9f11ea6cb48dc426b8047952073/INSTALL.md).
