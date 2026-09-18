"""Command line entry point: ``python -m research_sdk.sim``."""

from __future__ import annotations

import argparse
import signal
import sys
import threading
from collections.abc import Sequence

from research_sdk.config import (
    GRSIM_COMMAND_PORT,
    GRSIM_VISION_PORT,
    MULTICAST_INTERFACE_IP,
)
from research_sdk.sim.engine import SimConfig
from research_sdk.sim.server import (
    DEFAULT_VISION_GROUP,
    READY_MARKER,
    TIME_SCALES,
    ServerConfig,
    SimServer,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Lightweight grSim-compatible simulator (kinematic, accelerated time)."
    )
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=GRSIM_COMMAND_PORT)
    parser.add_argument(
        "--vision-address",
        default=DEFAULT_VISION_GROUP,
        help="Multicast group (default, like grSim) or a unicast IP such as 127.0.0.1",
    )
    parser.add_argument("--vision-port", type=int, default=GRSIM_VISION_PORT)
    parser.add_argument("--multicast-interface", default=MULTICAST_INTERFACE_IP)
    parser.add_argument("--vision-hz", type=float, default=60.0)
    parser.add_argument("--physics-hz", type=float, default=240.0)
    parser.add_argument(
        "--time-scale",
        type=int,
        choices=TIME_SCALES,
        default=1,
        help="Target simulated-time multiplier (default: 1; use headless above 5x)",
    )
    parser.add_argument("--robots-per-team", type=int, default=6)
    parser.add_argument("--noise-mm", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--duration", type=float, default=None, help="Stop after N seconds")
    parser.add_argument(
        "--stop-on-stdin-eof",
        action="store_true",
        help="Exit when stdin closes (used by the UI so a crashed UI never leaves the sim behind)",
    )
    return parser


def _watch_stdin(server: SimServer) -> None:
    try:
        while sys.stdin.read(1024):
            pass
    except (OSError, ValueError):
        pass
    server.stop()


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        server = SimServer(
            ServerConfig(
                command_host=args.command_host,
                command_port=args.command_port,
                vision_address=args.vision_address,
                vision_port=args.vision_port,
                multicast_interface=args.multicast_interface,
                vision_hz=args.vision_hz,
                physics_hz=args.physics_hz,
                time_scale=args.time_scale,
            ),
            SimConfig(
                robots_per_team=args.robots_per_team,
                noise_mm=args.noise_mm,
                seed=args.seed,
            ),
        )
    except (OSError, ValueError) as exc:
        print(f"[sim] failed to start: {exc}", flush=True)
        return 1

    def _stop(*_):
        server.stop()

    for name in ("SIGINT", "SIGTERM", "SIGBREAK"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), _stop)
    if args.stop_on_stdin_eof:
        threading.Thread(target=_watch_stdin, args=(server,), daemon=True).start()
    host, port = server.command_address
    print(
        f"{READY_MARKER} commands on {host}:{port}, vision to "
        f"{args.vision_address}:{args.vision_port} at {args.vision_hz:g} Hz, "
        f"target {args.time_scale}x",
        flush=True,
    )
    try:
        server.run(args.duration)
    finally:
        server.close()
        print(f"[sim] stopped after {server.frames_sent} frames", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
