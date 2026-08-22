"""Measure completed SSL-Vision frame latency to the planner input boundary."""

from __future__ import annotations

import argparse
from statistics import median

from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
from research_sdk.world.pipeline import VisionWorldPipeline


def camera_packet(camera_id: int):
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    detection = packet.detection
    detection.t_capture = 1.0
    detection.t_sent = 1.0
    detection.camera_id = camera_id
    robot = detection.robots_yellow.add()
    robot.confidence = 1.0
    robot.robot_id = camera_id
    robot.x = float(camera_id * 100)
    robot.y = 0.0
    robot.orientation = 0.0
    robot.pixel_x = 0.0
    robot.pixel_y = 0.0
    return packet


def percentile(values: list[float], percentage: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(len(ordered) * percentage))
    return ordered[index]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=2000)
    args = parser.parse_args()
    pipeline = VisionWorldPipeline(cameras=4)
    packets = [camera_packet(camera_id) for camera_id in range(4)]
    processing = []
    assembly = []

    for frame_number in range(1, args.frames + 101):
        update = None
        for packet in packets:
            packet.detection.frame_number = frame_number
            packet.detection.t_capture = float(frame_number)
            packet.detection.t_sent = float(frame_number)
            update = pipeline.ingest_bytes(packet.SerializeToString())
        if frame_number <= 100:
            continue
        processing.append(update.processing_latency_ms)
        assembly.append(update.frame_assembly_latency_ms)

    print(f"frames={args.frames}")
    print(f"final_packet_to_planner_median_ms={median(processing):.6f}")
    print(f"final_packet_to_planner_p95_ms={percentile(processing, 0.95):.6f}")
    print(f"four_packet_frame_to_planner_median_ms={median(assembly):.6f}")
    print(f"four_packet_frame_to_planner_p95_ms={percentile(assembly, 0.95):.6f}")


if __name__ == "__main__":
    main()
