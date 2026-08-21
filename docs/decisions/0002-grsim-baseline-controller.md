# 0002: grSim adaptor, baseline controller first

Date: 2026-08-01

## Context

We want to drive a robot in grSim (RoboCup Small Size League simulator) by sending a target position and letting it close the loop, following the approach in *Position Control of an Omnidirectional Mobile Robot* (Abbenseth & Ommer). That paper's actual contribution is a learned policy: Relative Entropy Policy Search (REPS) producing a Gaussian Process policy over (Δx, Δy, Δθ, ẋ, ẏ, θ̇) → (ẋ_d, ẏ_d, θ̇_d), trained offline in Matlab against sampled rollouts. It also explicitly starts from and compares against a simpler baseline: "the currently implemented approach is a PD-controller for each action with manually tuned parameters."

## Decisions

1. **Build the baseline first, not the learned policy.** `GrSimAdaptor` (`adaptors/grsim.py`) implements a proportional controller: position/heading error is computed in the global frame, rotated into the robot's local frame, scaled by a gain, clamped to a max speed, and sent as `veltangent`/`velnormal`/`velangular`. This is deliberately the paper's "not yet optimal" starting point (expect some overshoot/overcontrol per the paper's own findings), not its REPS/GP result. Reimplementing REPS + a squashed-kernel Gaussian Process policy (reward design, rollout sampling, dual optimization for α/η) is a real multi-stage research project on its own — worth its own ADR and phase once the baseline round-trip against real grSim is proven out.

2. **Protobuf bindings are compiled from the actual grSim/SSL `.proto` files, not hand-typed.** They live in `src/research_sdk/proto/*.proto` (grSim command/replacement/robot-status protocol, plus the SSL vision detection/geometry/wrapper protocol) and are compiled via `scripts/generate_protos.py` (`grpc_tools.protoc`) into `src/research_sdk/proto/generated/`. Guessing field numbers from memory was explicitly rejected — a wrong field number produces a packet grSim might partially accept or silently ignore, which is a much worse failure mode than a wrong port (which just times out, visibly).

3. **Generated proto modules' cross-imports are fixed with a contained sys.path trick.** protoc emits flat imports between generated files (`import grSim_Commands_pb2`, not a relative import), which only resolve if that directory is literally on `sys.path`. `proto/generated/__init__.py` inserts its own directory into `sys.path` once, then re-exports the specific message classes the rest of the SDK needs. This is the standard, well-known workaround for protobuf-python's codegen not being package-aware — contained to one file, nothing downstream needs to know about it.

4. **`GrSimAdaptor` is registered in `AdaptorRegistry` and exposed over MCP, but constructed lazily.** Every constructor argument has a network-local default, so — unlike `APIAdaptor` with an arbitrary `base_url` — there's no caller-supplied target to abuse; it's safe to expose. But its constructor has a real side effect (binds a UDP socket, joins a multicast group), unlike `APIAdaptor`'s lazy `httpx.Client`. `mcp/server.py` builds it on first tool call, not at import time, so `import research_sdk.mcp.server` never fails just because grSim or the right network interface isn't available yet.

5. **MATLAB link stays a separate concern (`FileBridgeAdaptor`, ADR-adjacent but not merged in).** The paper's own Matlab↔central-software link is a file-based request/response handoff (section 5.2) — `FileBridgeAdaptor` already implements that generically. When REPS/GP training work starts, the training loop lives in Matlab and talks to this SDK through that bridge, not through a new mechanism.

## Status

Accepted. Next phase (separate ADR when it starts): reward function + rollout sampling against `GrSimAdaptor`, squashed-ExpQuad-plus-linear kernel, dual optimization, all driven from Matlab via `FileBridgeAdaptor`, replacing the P-controller's gains with the learned policy's output.
