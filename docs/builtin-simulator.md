# Built-in simulator

The console can run its own lightweight simulator in the background. The
simulator speaks the grSim UDP protocol on the ports configured in
`config/network_input.yaml`, so the rest of the console works against it
unchanged. That includes Load from grSim, Apply to grSim, Execution, live
replanning, and the reroute-gate and prediction switches.

## Use it

1. In **Configurations**, keep `grsim_command_ip: 127.0.0.1`.
2. Stop any other grSim (Docker, WSL or native) that uses the same ports.
3. Tick **Built-in simulator** in the toolbar. The status changes to
   *built-in (running)* and six robots per team appear, as in a fresh grSim.

Before starting it, choose **1x**, **2x**, or **5x** from the adjacent Speed
selector. The UI simulator is deliberately capped at 5x. Stop the simulator
before changing speed.

Untick it to go back to an external grSim. Closing the console stops the
simulator. The simulator also exits by itself if the console crashes.

Without the UI:

```shell
research-sdk-sim                      # uses network_input.yaml ports
python -m research_sdk.sim --robots-per-team 3 --noise-mm 2
python -m research_sdk.sim --vision-address 127.0.0.1   # unicast instead of multicast
python -m research_sdk.sim --time-scale 5
```

## What it simulates

| | Built-in simulator | grSim (ODE) |
|---|---|---|
| Robot motion | Disc tracking the commanded body-frame velocity, with acceleration and speed limits | Wheels, motors, friction |
| Contacts | Robots pushed apart; ball pushed and flat-kicked | Rigid-body physics |
| Vision | 4 quadrant cameras, 60 Hz, optional Gaussian noise | 4 cameras, configurable noise |
| Commands | `grSim_Packet` commands and replacements (robots, ball, turn-on) | Same |
| Stale commands | Robot stops after 0.5 s without a command | Set by grSim |
| Platform | Pure Python, any OS | Needs a grSim build (WSL, Docker or native) |

Use it for UI work, demos and quick closed-loop checks. Use grSim
([physics.md](physics.md)) when results must come from rigid-body physics.

Acceleration does not enlarge the 1/240 s physics step, so contacts do not
gain extra numerical tunnelling from the speed control. Vision remains capped
at 60 wall-clock frames/s to avoid flooding the UI. Consequently, high-speed
live runs give the external controller fewer observations per simulated second;
use the deterministic headless backend for research comparisons, and use 1x
for final closed-loop validation.

The live controller updates commands every 50 ms of wall time. That interval
represents 0.25 s of virtual motion at 5x, so short waypoints can still be more
sensitive than at 1x. Rates above 5x are deliberately unavailable in the UI
simulator: supporting them correctly requires running vision, control, and
planning on the same virtual clock, not merely accelerating the simulator
process.

A closed-loop check on `scenarios/crowded.json` ran the Voronoi planner through
the real runtime (commands → simulator → vision → runtime). All three robots
arrived in 11.7 s with 57–59 mm final error. The earlier grSim validation run
took 12.2 s.

## Design

- `research_sdk/sim/engine.py` holds the model (`SimWorld`) and the protocol boundary:
  - `apply_packet()` takes a `grSim_Packet`.
  - `detection_packets()` produces one frame's four `SSL_WrapperPacket`s.
- `research_sdk/sim/server.py` schedules fixed 240 Hz virtual physics at the selected time scale, sends vision at 60 wall-clock Hz, and polls a non-blocking command socket.
- `research_sdk/sim/manager.py` runs the simulator as a child process, so the physics loop doesn't compete with the UI thread.
  - It waits for the `[sim] ready` line.
  - It stops the simulator by closing its stdin, which also happens if the UI dies.
- `research_sdk/ui/sim_control.py` is the toolbar action, status label and watchdog.

Vision goes to the multicast group from `ssl_vision_multicast_group` on
`grsim_vision_port`, with loopback enabled, exactly as grSim sends it. Every
console listener receives it.
