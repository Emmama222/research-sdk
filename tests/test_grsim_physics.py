"""Packet boundary tests and opt-in tests against the actual grSim/ODE binary."""

import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from research_sdk.grsim_physics import (
    FrameAssembler,
    GrSimSession,
    PhysicsError,
    _profile,
    local_velocity,
    simulate_physics,
    validate_scenario,
    velocity_packet,
)
from research_sdk.headless import (
    SimulationConfig,
    main,
    run_experiments,
    simulate,
    summarize_results,
)
from research_sdk.network.proto2.ssl_vision_detection_pb2 import SSL_DetectionFrame
from research_sdk.ui.scenarios import Scenario, ScenarioBall, ScenarioObstacle, ScenarioRobot


def detection(frame, camera, stamp):
    return SSL_DetectionFrame(frame_number=frame, camera_id=camera, t_capture=stamp, t_sent=stamp)


def test_frames_are_not_combined_across_physics_ticks():
    assembler = FrameAssembler()
    for camera in (0, 1, 2):
        assert assembler.push(detection(1, camera, 0.1)) is None
    for camera in (0, 1, 2):
        assert assembler.push(detection(2, camera, 0.2)) is None
    assert assembler.push(detection(1, 3, 0.1)) is None
    result = assembler.push(detection(2, 3, 0.2))
    assert result.frame == 2
    assert result.time_s == 0.2
    assert assembler.incomplete == 1
    assert assembler.push(detection(2, 3, 0.2)) is None


def test_camera_timestamp_mismatch_rejected():
    assembler = FrameAssembler()
    for camera in range(3):
        assembler.push(detection(1, camera, 0.1))
    with pytest.raises(PhysicsError, match="timestamps disagree"):
        assembler.push(detection(1, 3, 0.2))


def test_local_velocity_units_and_heading():
    velocity = local_velocity(1000, 0, math.pi / 2)
    assert velocity == pytest.approx((0, -1, 0))
    packet = velocity_packet(True, {2: velocity})
    assert packet.IsInitialized()
    assert packet.commands.isteamyellow
    command = packet.commands.robot_commands[0]
    assert command.id == 2
    assert command.velnormal == pytest.approx(-1)
    assert not command.wheelsspeed


def test_profile_geometry_interval_is_safe_for_four_cameras(tmp_path):
    path = tmp_path / "grsim.xml"
    _profile(path, list(range(41000, 41007)), 2, 1 / 120)
    root = ET.parse(path).getroot()
    assert root.tag == "VarXML"
    values = {var.attrib["name"]: var.text for var in root.iter("Var")}
    assert int(values["Send geometry every X frames"]) // 4 > 0
    assert float(values["ODE time step"]) == pytest.approx(1 / 120)
    assert values["Vision multicast address"] == "127.0.0.1"


def test_nonfinite_ball_telemetry_rejected():
    assembler = FrameAssembler()
    for camera in range(3):
        assembler.push(detection(1, camera, 0.1))
    part = detection(1, 3, 0.1)
    part.balls.add(confidence=1, x=float("nan"), y=0, pixel_x=0, pixel_y=0)
    with pytest.raises(PhysicsError, match="ball position"):
        assembler.push(part)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 0, -1])
def test_nonfinite_time_steps_are_rejected(bad):
    with pytest.raises(ValueError):
        SimulationConfig(dt_s=bad)


def straight():
    return Scenario(
        "physics-straight",
        robots=[ScenarioRobot(0, True, (-1000, -500), (1000, -500), math.pi / 2)],
    )


def test_unsupported_geometry_rejected_before_launch():
    scenario = straight()
    scenario.obstacles.append(ScenarioObstacle(1, False, (0, 0), 200))
    with pytest.raises(ValueError, match="90 mm"):
        validate_scenario(scenario, "visibility")
    scenario.obstacles[0] = ScenarioObstacle(0, True, (0, 0), 90)
    with pytest.raises(ValueError, match="Duplicate"):
        validate_scenario(scenario, "visibility")


def test_invalid_ball_input_rejected_before_launch():
    scenario = straight()
    scenario.ball = ScenarioBall((0, 0), (float("nan"), 0))
    with pytest.raises(ValueError, match="finite 2D"):
        validate_scenario(scenario, "visibility")
    scenario.ball = ScenarioBall((0, 4000))
    with pytest.raises(ValueError, match="within the playable field"):
        validate_scenario(scenario, "visibility")


def test_kinematics_never_counts_as_a_physics_validation_pass():
    from dataclasses import replace

    result = simulate(straight(), "visibility")
    assert result.completed and not result.physics_validation_passed
    physical = replace(result, backend="grsim", physics_validation_passed=True)
    summary = summarize_results([result, physical])
    assert len(summary) == 2
    assert [row["physics_validation_passes"] for row in summary] == [0, 1]


def test_native_batch_routes_without_kinematic_fallback(monkeypatch, tmp_path):
    calls = []

    def native_trial(scenario, planner, **kwargs):
        calls.append(kwargs)
        raise PhysicsError("Engine unavailable")

    monkeypatch.setattr("research_sdk.grsim_physics.simulate_physics", native_trial)
    with pytest.raises(PhysicsError, match="Engine unavailable"):
        run_experiments([straight()], ["visibility"], backend="grsim", evidence_dir=tmp_path)
    assert calls[0]["config"].dt_s == pytest.approx(1 / 120)


def test_completed_but_invalid_native_run_exits_nonzero(monkeypatch, tmp_path):
    from dataclasses import replace

    result = replace(simulate(straight(), "visibility"), backend="grsim", missed_frames=1)
    monkeypatch.setattr("research_sdk.grsim_physics.simulate_physics", lambda *a, **kw: result)
    scenario_file = tmp_path / "input.json"
    scenario_file.write_text(json.dumps(straight().to_dict()))
    output = tmp_path / "output"
    assert (
        main(
            [
                str(scenario_file),
                "--backend",
                "grsim",
                "--planner",
                "visibility",
                "--output-dir",
                str(output),
            ]
        )
        == 2
    )
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["physics_validation_passed"] is False
    assert (output / "runs.csv").is_file()


native = pytest.mark.skipif(
    os.environ.get("RESEARCH_RUN_PHYSICS") != "1",
    reason="Set RESEARCH_RUN_PHYSICS=1 in Linux/WSL after scripts/build_grsim.sh",
)
BINARY = Path(".local/grsim/bin/grSim")


@native
def test_native_closed_loop_reaches_and_stops_with_evidence(tmp_path):
    result = simulate_physics(
        straight(),
        "visibility",
        config=SimulationConfig(dt_s=1 / 120),
        binary=BINARY,
        evidence_dir=tmp_path,
    )
    assert result.completed, result.to_record()
    assert result.physics_validation_passed, result.to_record()
    assert result.max_final_error_mm <= 60
    assert result.max_final_speed_mmps <= 50
    folder = Path(result.evidence_directory)
    assert (folder / "trajectory.jsonl").stat().st_size > 1000
    provenance = json.loads((folder / "provenance.json").read_text())
    assert provenance["physics"] == "ODE"
    assert provenance["build"]["revision"] == "fe2bd2915a46f9f11ea6cb48dc426b8047952073"


@native
def test_native_motor_response_and_body_collision(tmp_path):
    scenario = Scenario(
        "contact",
        robots=[ScenarioRobot(0, True, (-600, 0), (1000, 0))],
        obstacles=[ScenarioObstacle(1, False, (0, 0), 90)],
    )
    obstacles, keys, count = validate_scenario(scenario, "visibility")
    with GrSimSession(BINARY, tmp_path / "contact", count, 1 / 120) as engine:
        start = engine.place(scenario, obstacles, keys)
        previous = start
        first_speed = None
        minimum_separation = 1000
        while previous.time_s - start.time_s < 1.0:
            engine.send(velocity_packet(True, {0: (3, 0, 0)}))
            engine.send(velocity_packet(False, {1: (0, 0, 0)}))
            current = engine.read()
            position = current.robots[True, 0][:2]
            if first_speed is None:
                first_speed = math.dist(position, previous.robots[True, 0][:2]) / (
                    current.time_s - previous.time_s
                )
            minimum_separation = min(
                minimum_separation, math.dist(position, current.robots[False, 1][:2])
            )
            previous = current
        # Motor torque accelerates a body; it cannot jump to commanded speed.
        assert first_speed < 1500
        # Physical bodies touch and transfer force instead of passing through.
        assert 150 < minimum_separation < 190
        assert current.robots[False, 1][0] > 20
        assert current.robots[True, 0][0] < current.robots[False, 1][0]


@native
def test_native_ball_friction_slows_a_rolling_ball(tmp_path):
    scenario = straight()
    scenario.ball = ScenarioBall((-1500, 1000), (1500, 0))
    obstacles, keys, count = validate_scenario(scenario, "visibility")
    with GrSimSession(BINARY, tmp_path / "ball", count, 1 / 120) as engine:
        start = engine.place(scenario, obstacles, keys)
        previous = engine.read()
        samples = []
        while previous.time_s - start.time_s < 1.0:
            current = engine.read()
            if current.ball is not None and previous.ball is not None:
                speed = math.dist(current.ball, previous.ball) / (current.time_s - previous.time_s)
                samples.append(speed)
            previous = current
        assert len(samples) > 80
        assert max(samples[:20]) > 1000
        assert sum(samples[-10:]) / 10 < sum(samples[10:20]) / 10 - 100
