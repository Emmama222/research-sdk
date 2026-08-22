"""Synchronous, reusable vision-to-planner world-state pipeline."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from time import perf_counter_ns, time

from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
from research_sdk.process_workers.vision_runner import VisionFrameAssembler
from research_sdk.world.map.world_map import WorldMap
from research_sdk.world.scene import PlanningScene
from research_sdk.world.snapshot import WorldSnapshot, world_snapshot_from_frame

SnapshotListener = Callable[[WorldSnapshot], None]


class WorldSnapshotStore:
    """Single-owner store that publishes immutable snapshots to consumers."""

    def __init__(self) -> None:
        self._snapshot: WorldSnapshot | None = None
        self._listeners: list[SnapshotListener] = []

    @property
    def current(self) -> WorldSnapshot | None:
        return self._snapshot

    def publish(self, snapshot: WorldSnapshot) -> None:
        self._snapshot = snapshot
        for listener in tuple(self._listeners):
            listener(snapshot)

    def subscribe(self, listener: SnapshotListener) -> Callable[[], None]:
        self._listeners.append(listener)

        def unsubscribe() -> None:
            if listener in self._listeners:
                self._listeners.remove(listener)

        return unsubscribe


@dataclass(frozen=True, slots=True)
class WorldPipelineUpdate:
    """Output boundary ready for both world and planner consumers."""

    snapshot: WorldSnapshot
    planning_scene: PlanningScene
    processing_latency_ms: float
    mapping_time_ms: float
    frame_assembly_latency_ms: float


class VisionWorldPipeline:
    """Convert raw SSL-Vision packets into snapshots and planner scenes.

    The class is deliberately synchronous. Call it directly from a QThread, or
    place it behind a multiprocessing Queue without changing its consumers.
    """

    def __init__(self, cameras: int = 4, *, store: WorldSnapshotStore | None = None) -> None:
        self.assembler = VisionFrameAssembler(cameras)
        self.store = store or WorldSnapshotStore()
        self.world_map = WorldMap()
        self.latest_scene: PlanningScene | None = None
        self._frame_started_ns: int | None = None

    def ingest(self, packet, *, _entered_ns: int | None = None) -> WorldPipelineUpdate | None:
        """Return an update only when all camera packets complete a frame."""
        entered_ns = perf_counter_ns() if _entered_ns is None else _entered_ns
        if packet is None or not packet.HasField("detection"):
            return None
        detection = packet.detection
        if self._frame_started_ns is None:
            self._frame_started_ns = entered_ns
        assembly_started_ns = self._frame_started_ns

        frame = self.assembler.push(detection)
        if frame is None:
            return None

        version = 1 if self.store.current is None else self.store.current.version + 1
        snapshot = world_snapshot_from_frame(frame, version=version)
        self.store.publish(snapshot)
        received_at_s = time()
        mapping_started_ns = perf_counter_ns()
        self.world_map.update(snapshot, received_at_s=received_at_s)
        scene = self.world_map.planning_scene(now_s=received_at_s)
        mapping_finished_ns = perf_counter_ns()
        self.latest_scene = scene
        finished_ns = perf_counter_ns()
        processing_latency_ms = (finished_ns - entered_ns) / 1_000_000.0
        frame_assembly_latency_ms = (finished_ns - assembly_started_ns) / 1_000_000.0
        self._frame_started_ns = entered_ns if self.assembler.frame is not None else None
        return WorldPipelineUpdate(
            snapshot=snapshot,
            planning_scene=scene,
            processing_latency_ms=processing_latency_ms,
            mapping_time_ms=(mapping_finished_ns - mapping_started_ns) / 1_000_000.0,
            frame_assembly_latency_ms=frame_assembly_latency_ms,
        )

    def ingest_bytes(self, payload: bytes) -> WorldPipelineUpdate | None:
        """Decode one UDP payload and carry it through to the planner boundary."""
        entered_ns = perf_counter_ns()
        packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket.FromString(payload)
        return self.ingest(packet, _entered_ns=entered_ns)
