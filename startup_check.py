"""Verify the Research SDK grSim-to-planner startup path."""

from __future__ import annotations

import argparse
import socket
import struct
import sys
from dataclasses import dataclass
from time import monotonic

from google.protobuf.message import DecodeError

from research_sdk.config import (
    GRSIM_COMMAND_IP,
    GRSIM_COMMAND_PORT,
    GRSIM_VISION_PORT,
    MULTICAST_INTERFACE_IP,
    SSL_VISION_MULTICAST_GROUP,
)
from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
from research_sdk.world.pipeline import VisionWorldPipeline, WorldPipelineUpdate

PASS = "[PASS]"
FAIL = "[FAIL]"
WARN = "[WARN]"


@dataclass(frozen=True, slots=True)
class VisionProbeResult:
    packets: int
    bytes_received: int
    camera_ids: tuple[int, ...]
    update: WorldPipelineUpdate | None
    decode_errors: int = 0


def header(title: str) -> None:
    print(f"\n{'─' * 62}\n  {title}\n{'─' * 62}")


def configured_endpoints() -> bool:
    header("1. Configured endpoints")
    print(f"  {PASS} grSim vision:  {SSL_VISION_MULTICAST_GROUP}:{GRSIM_VISION_PORT}")
    print(f"  {PASS} Join interface: {MULTICAST_INTERFACE_IP}")
    print(f"  {PASS} grSim commands: {GRSIM_COMMAND_IP}:{GRSIM_COMMAND_PORT}")
    return True


def protobuf_check() -> bool:
    header("2. Protobuf and pipeline imports")
    try:
        packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
        VisionWorldPipeline(cameras=4)
    except (ImportError, RuntimeError, ValueError) as exc:
        print(f"  {FAIL} Cannot initialize vision pipeline: {exc}")
        return False
    print(f"  {PASS} SSL_WrapperPacket available: {type(packet).__name__}")
    print(f"  {PASS} VisionWorldPipeline initialized")
    return True


def open_multicast_socket(group: str, port: int, interface_ip: str) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    membership = struct.pack(
        "=4s4s",
        socket.inet_aton(group),
        socket.inet_aton(interface_ip),
    )
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
    return sock


def receive_grsim_vision(timeout_s: float) -> VisionProbeResult:
    pipeline = VisionWorldPipeline(cameras=4)
    packets = 0
    bytes_received = 0
    decode_errors = 0
    camera_ids: set[int] = set()
    update = None
    sock = open_multicast_socket(
        SSL_VISION_MULTICAST_GROUP,
        GRSIM_VISION_PORT,
        MULTICAST_INTERFACE_IP,
    )
    deadline = monotonic() + timeout_s
    try:
        while monotonic() < deadline and update is None:
            sock.settimeout(max(0.05, deadline - monotonic()))
            try:
                payload, _sender = sock.recvfrom(65535)
            except TimeoutError:
                break
            packets += 1
            bytes_received += len(payload)
            try:
                packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket.FromString(payload)
            except DecodeError:
                decode_errors += 1
                continue
            if packet.HasField("detection"):
                camera_ids.add(int(packet.detection.camera_id))
            update = pipeline.ingest(packet)
    finally:
        sock.close()
    return VisionProbeResult(
        packets=packets,
        bytes_received=bytes_received,
        camera_ids=tuple(sorted(camera_ids)),
        update=update,
        decode_errors=decode_errors,
    )


def vision_check(timeout_s: float) -> bool:
    header("3. grSim vision: UDP → snapshot → planner")
    try:
        result = receive_grsim_vision(timeout_s)
    except OSError as exc:
        print(f"  {FAIL} Could not join or read multicast: {exc}")
        print("         Check multicast_interface_ip in network_input.yaml.")
        return False

    if result.packets == 0:
        print(
            f"  {FAIL} No UDP packets received in {timeout_s:.1f}s from "
            f"{SSL_VISION_MULTICAST_GROUP}:{GRSIM_VISION_PORT}"
        )
        print("         grSim can be open without publishing vision on this endpoint.")
        return False

    print(
        f"  {PASS} UDP input: {result.packets} packets, "
        f"{result.bytes_received} bytes, cameras={result.camera_ids or 'none'}"
    )
    if result.decode_errors:
        print(f"  {WARN} Protobuf decode errors: {result.decode_errors}")
    if result.update is None:
        print(f"  {FAIL} Packets arrived but no usable world snapshot was produced")
        return False

    snapshot = result.update.snapshot
    robots = (*snapshot.our_robots, *snapshot.their_robots)
    print(
        f"  {PASS} WorldSnapshot v{snapshot.version}: "
        f"frame={snapshot.frame_number}, robots={len(robots)}, "
        f"ball={'yes' if snapshot.ball is not None else 'no'}"
    )
    print(f"  {PASS} Planner scene: {len(result.update.planning_scene.obstacles)} obstacles")
    print(
        f"  {PASS} Internal latency: final packet → planner "
        f"{result.update.processing_latency_ms:.3f} ms"
    )
    if not robots and snapshot.ball is None:
        print(f"  {WARN} Snapshot is valid but contains no visible robots or ball")
    return True


def command_route_check() -> bool:
    header("4. grSim command route (no packet sent)")
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((GRSIM_COMMAND_IP, GRSIM_COMMAND_PORT))
        local_ip, local_port = sock.getsockname()
    except OSError as exc:
        print(f"  {FAIL} No UDP route to grSim command endpoint: {exc}")
        return False
    finally:
        if sock is not None:
            sock.close()
    print(
        f"  {PASS} Route ready: {local_ip}:{local_port} → "
        f"{GRSIM_COMMAND_IP}:{GRSIM_COMMAND_PORT}"
    )
    print("         UDP has no acknowledgement; this does not prove grSim received it.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="seconds to wait for a usable grSim world snapshot",
    )
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    print("\nResearch SDK startup verification")
    results = {
        "config": configured_endpoints(),
        "protobuf": protobuf_check(),
        "vision_pipeline": vision_check(args.timeout),
        "command_route": command_route_check(),
    }
    header("Summary")
    for name, passed in results.items():
        print(f"  {PASS if passed else FAIL} {name}")
    failed = [name for name, passed in results.items() if not passed]
    if failed:
        print(f"\n  Failed required checks: {', '.join(failed)}")
        return 1
    print("\n  grSim data reached the planner boundary successfully.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
