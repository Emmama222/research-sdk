#!/usr/bin/env python3
"""Drive grSim from the planners, headless, so you can watch them work.

Places a blue team on one touchline and sends it across the pitch and back,
planning every control tick with whichever backend you pick. The yellow team
patrols across the middle as moving obstacles, so the blue robots have to keep
replanning around traffic rather than driving a straight line once.

Everything the study measures is printed live while it runs: replans, how often
a replan reverses the robot, planning time, and closest approach to another
robot. Watching the pitch shows you the behaviour; the numbers say whether it is
the behaviour the paper claims.

Run grSim first. Both this and grSim must be on the same side of the WSL
boundary, because WSL2 NAT does not carry multicast to Windows.

Obstacles follow one of dynamic_scenario.py's own two patrol-loop layouts
(--scenario 1 or 2), at the same default speed (1.0 m/s) it uses offline, so
a live run and an offline run share obstacle geometry, not just a count.

Pass --samples N to run N discrete, resettable trials instead of one
continuous session -- start/goal, arrival tolerance, contact threshold, and
per-trial timeout all match dynamic_scenario.py's own constants, and each
trial draws a random obstacle starting phase the same way
run_all_samples() does offline, so the two are a fair paired comparison.

Usage:
    python scripts/drive_grsim.py --planner voronoi --scenario 1
    python scripts/drive_grsim.py --planner visibility --scenario 2 --duration 60
    python scripts/drive_grsim.py --planner voronoi --scenario 2 --samples 200 --out-json results/grsim_voronoi_s2.json
    python scripts/drive_grsim.py --selftest
"""

from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from path_stability import Stability
from dynamic_scenario import (
    ARRIVED_MM as OFFLINE_ARRIVED_MM,
    CONTACT_MM as OFFLINE_CONTACT_MM,
    GOAL_MM as OFFLINE_GOAL_MM,
    SCENARIO_1,
    SCENARIO_2,
    START_MM as OFFLINE_START_MM,
)
from static_scenario import (
    SCENARIOS as STATIC_SCENARIOS,
    path_length_m,
    path_safety_m,
)
from demo_planners import _run_prm, _run_visibility, _run_voronoi

from research_sdk.network.command_dispatcher import RobotCommandDispatcher
from research_sdk.network.grSimPacketFactory import grSimPacketFactory
from research_sdk.network.robot_command import RobotCommand
from research_sdk.network.ssl_sockets import grSimSender, grSimVision
from research_sdk.planners.reroute import (
    RouteState,
    commit_reroute,
    evaluate_route,
    note_no_reroute,
)
from research_sdk.planners.common import (
    DEFAULT_ROBOT_RADIUS_MM,
    Obstacle,
    PlanRequest,
)
from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.planners.PRM import prm_dijkstra
from research_sdk.planners.VisibilityGraph import visibility_graph
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene

Point = tuple[float, float]

CONTROL_TICK_S = 0.05        # ui/execution/page.py's own execution timer

# grSim does NOT hold a velocity command between packets. Measured on a robot
# commanded at a steady 1.2 m/s for 3 s: sending once per 50 ms control tick
# moved it 22 mm, sending at 50 Hz moved it 618 mm, and neither approached the
# 3600 mm the command asks for. The robot accelerates while commands arrive and
# coasts down between them. So planning runs at the control tick and delivery
# runs far faster, through the SDK's own RobotCommandDispatcher, which repeats
# the latest command and zeroes it after command_ttl_s of silence.
COMMAND_HZ = 100.0
ARRIVED_MM = 150.0           # close enough to call it, and to turn around
CONTACT_MM = 2 * DEFAULT_ROBOT_RADIUS_MM   # centre distance at which robots touch
INVALIDATION_MM = 90.0       # the source study's replan trigger threshold

# Keep well inside the touchlines: a robot commanded past them piles into the
# wall and stops, which looks like a planner failure and is not one.
START_X, GOAL_X = -3200.0, 3200.0
LANE_SPACING_MM = 900.0

# The same two obstacle layouts dynamic_scenario.py tests offline -- seven
# obstacles (scenario 1) or five (scenario 2), each patrolling a closed
# rectangular loop. Picking one here means the live run and the offline run
# share identical obstacle geometry, not just an obstacle count.
SCENARIOS = {"1": SCENARIO_1, "2": SCENARIO_2}


def _prm(request, key):
    # dynamic_scenario.py's own PRM wrapper passes a fresh seed on every
    # replan (the running recalculated count) so each call samples a
    # genuinely different roadmap, same as real non-deterministic PRM
    # behaviour. Without this, prm_dijkstra.plan()'s seed defaults to 0 on
    # every call, so every tick rebuilds the IDENTICAL 40-point roadmap --
    # if that one fixed roadmap doesn't connect start to goal, every single
    # call fails forever, which is what a fixed-seed=0 caller (this
    # function, before this fix) produced under scenario 1's geometry.
    seed = key if isinstance(key, int) else 0
    return prm_dijkstra.plan(request, seed=seed, skip_direct_path=False, search_key=key)


def _visibility(request, key):
    return visibility_graph.plan(request, skip_direct_path=False, search_key=key)


PLANNERS = {
    "prm": _prm,
    "visibility": _visibility,
    # Voronoi takes a scene rather than a PlanRequest, so it is adapted in
    # `plan_for` rather than here.
    "voronoi": None,
}


@dataclass
class Robot:
    """One blue robot under our control."""

    robot_id: int
    target: Point
    path: tuple[Point, ...] = ()
    stability: Stability = field(default_factory=Stability)
    replans: int = 0
    plan_ms_total: float = 0.0
    laps: int = 0
    closest_mm: float = float("inf")


def rotate_into_robot_frame(ex: float, ey: float, theta: float) -> tuple[float, float]:
    """World-frame error to the robot's own axes.

    grSim reads veltangent and velnormal in the robot's frame, so a world-frame
    velocity sent unrotated drives every robot in a different direction
    depending on which way it happens to be facing.
    """
    c, s = math.cos(theta), math.sin(theta)
    return c * ex + s * ey, -s * ex + c * ey


def path_blocked(path, obstacles, threshold_mm: float = INVALIDATION_MM) -> bool:
    """The source study's trigger: any obstacle within 90 mm of the path."""
    if len(path) < 2:
        return False
    for o in obstacles:
        for a, b in zip(path, path[1:]):
            if _point_to_segment(o.pos_mm, a, b) <= o.radius_mm + threshold_mm:
                return True
    return False


def _point_to_segment(p: Point, a: Point, b: Point) -> float:
    px, py = p
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    seg = dx * dx + dy * dy
    t = 0.0 if seg == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class Vision:
    """Latest pose per robot, kept current by draining every queued frame.

    The drain is not an optimisation. grSim emits four camera streams at ~60 Hz
    each and a 50 ms tick leaves a dozen frames waiting; reading one of them
    gives a pose from the previous tick, which reads as a robot that will not
    move. That mistake has cost this project two debugging sessions.
    """

    def __init__(self) -> None:
        self.sock = grSimVision(threading.Event())
        self.sock.is_running.set()
        self.blue: dict[int, tuple[float, float, float]] = {}
        self.yellow: dict[int, tuple[float, float, float]] = {}

    def update(self) -> int:
        self.sock.sock.setblocking(False)
        frames = 0
        try:
            while True:
                packet = self.sock.listen()
                if packet is None or not packet.HasField("detection"):
                    continue
                frames += 1
                for r in packet.detection.robots_blue:
                    self.blue[r.robot_id] = (r.x, r.y, r.orientation)
                for r in packet.detection.robots_yellow:
                    self.yellow[r.robot_id] = (r.x, r.y, r.orientation)
        except (BlockingIOError, OSError):
            pass
        finally:
            self.sock.sock.setblocking(True)
        return frames

    def wait_for_robots(self, blue: int, yellow: int, timeout_s: float = 5.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.update()
            if len(self.blue) >= blue and len(self.yellow) >= yellow:
                return True
            time.sleep(0.05)
        return False


def obstacles_for(vision: Vision, skip_blue: int) -> tuple[Obstacle, ...]:
    """Every other robot on the pitch, both teams."""
    out = []
    for rid, (x, y, _t) in vision.blue.items():
        if rid == skip_blue:
            continue
        out.append(Obstacle(pos_mm=(x, y), radius_mm=DEFAULT_ROBOT_RADIUS_MM,
                            robot_id=rid, isYellow=False))
    for rid, (x, y, _t) in vision.yellow.items():
        out.append(Obstacle(pos_mm=(x, y), radius_mm=DEFAULT_ROBOT_RADIUS_MM,
                            robot_id=rid, isYellow=True))
    return tuple(out)


def plan_for(planner: str, start: Point, goal: Point,
             obstacles: tuple[Obstacle, ...], key) -> tuple[tuple[Point, ...], float]:
    """One planning call. Returns the waypoints and how long it took, in ms."""
    t0 = time.perf_counter()
    if planner == "voronoi":
        scene = PlanningScene(
            timestamp=0.0,
            obstacles=tuple(
                PlanningObstacle(robot_id=o.robot_id, isYellow=o.isYellow,
                                 pos_mm=o.pos_mm, radius_mm=o.radius_mm)
                for o in obstacles
            ),
            field=FieldDimensions(),
        )
        result = VoronoiDijkstraPlanner().plan(scene, start, goal, search_key=key)
        waypoints = tuple(result.waypoints_mm)
    else:
        request = PlanRequest(start_mm=start, goal_mm=goal, obstacles=obstacles)
        result = PLANNERS[planner](request, key)
        # The visibility graph and PRM include the start as their first
        # waypoint; Voronoi does not. Drop it so every backend hands back the
        # same thing: where to go next.
        waypoints = tuple(result.waypoints_mm)[1:] if result.success else ()
    return waypoints, (time.perf_counter() - t0) * 1000.0


def place_teams(sender: grSimSender, blue: int, loops: tuple,
                 phases: list[float] | None = None,
                 start: Point | None = None) -> None:
    """Line the blue team up on one touchline (or at `start`, for a batch
    trial); place each yellow robot at arc length `phases[i]` (default 0)
    of its assigned patrol loop -- the same geometry and, with `phases`
    supplied, the same per-trial randomised starting point
    dynamic_scenario.py uses offline."""
    phases = phases if phases is not None else [0.0] * len(loops)
    robots = []
    for i in range(blue):
        sx, sy = start if start is not None else (START_X, 0.0)
        robots.append({
            "x": sx / 1000.0,
            "y": (sy + (i - (blue - 1) / 2.0) * LANE_SPACING_MM) / 1000.0,
            "orientation": 0.0, "robot_id": i, "isYellow": False,
        })
    for i, loop in enumerate(loops):
        x, y = loop.position(phases[i])
        robots.append({
            "x": x / 1000.0, "y": y / 1000.0,
            "orientation": 0.0, "robot_id": i, "isYellow": True,
        })
    sender.send_packet(grSimPacketFactory.scenario_replacement_command(robots))


def drive_obstacles(dispatcher: RobotCommandDispatcher, vision: Vision,
                     loops: tuple, obstacle_speed_mps: float, elapsed_s: float,
                     phases: list[float] | None = None) -> None:
    """Each yellow robot chases the point its assigned patrol loop is at
    right now, at the same speed and along the same closed-loop geometry
    dynamic_scenario.py uses offline (`phases[i] + speed * elapsed`, exactly
    its own obstacles_at()). Driven with the same pursuit controller as the
    blue robots, rather than teleported like the offline (synthetic-time)
    version -- grSim obstacles are real robots and have to be commanded,
    not placed."""
    phases = phases if phases is not None else [0.0] * len(loops)
    for rid, loop in enumerate(loops):
        pose = vision.yellow.get(rid)
        if pose is None:
            continue
        x, y, theta = pose
        target = loop.position(phases[rid] + obstacle_speed_mps * 1000.0 * elapsed_s)
        ex, ey = target[0] - x, target[1] - y
        dist = math.hypot(ex, ey)
        if dist > 1.0:
            scale = obstacle_speed_mps * min(1.0, dist / 400.0) / dist
            vx, vy = rotate_into_robot_frame(ex * scale, ey * scale, theta)
        else:
            vx = vy = 0.0
        dispatcher.publish(
            RobotCommand(robot_id=rid, vx=vx, vy=vy, w=0.0, isYellow=True)
        )


def run_trial(dispatcher: RobotCommandDispatcher, vision: Vision, sender: grSimSender,
              planner: str, trigger: str, period_ms: float, speed: float,
              loops: tuple, obstacle_speed_mps: float, phases: list[float],
              max_s: float) -> dict | None:
    """One discrete trial: reset the robot to dynamic_scenario.py's own
    start and each obstacle to `phases[i]` on its loop, drive until arrival
    or `max_s`, and return the same fields RunResult reports offline, so
    this trial and one offline sample are directly comparable. Returns
    None if the reset couldn't be confirmed (vision didn't see both teams
    in time) -- a setup failure, not a trial outcome, and shouldn't be
    counted as one.
    """
    place_teams(sender, 1, loops, phases=phases, start=OFFLINE_START_MM)
    time.sleep(0.3)
    if not vision.wait_for_robots(1, len(loops), timeout_s=8.0):
        return None

    stability = Stability()
    path: tuple = ()
    replans = 0
    plan_ms_total = 0.0
    collisions = 0
    in_contact = [False] * len(loops)
    ticks_since_plan = 0
    arrived = False
    route_state = RouteState()

    started = time.time()
    while True:
        elapsed = time.time() - started
        if elapsed >= max_s:
            break
        tick_start = time.perf_counter()
        vision.update()
        drive_obstacles(dispatcher, vision, loops, obstacle_speed_mps, elapsed, phases=phases)

        pose = vision.blue.get(0)
        if pose is None:
            time.sleep(CONTROL_TICK_S)
            continue
        x, y, theta = pose
        here = (x, y)

        # Rising-edge collision episodes, matching dynamic_scenario.py's own
        # accounting: true positions, once per contact, not once per tick.
        for i in range(len(loops)):
            opose = vision.yellow.get(i)
            if opose is None:
                continue
            touching = math.dist(here, opose[:2]) <= OFFLINE_CONTACT_MM
            if touching and not in_contact[i]:
                collisions += 1
            in_contact[i] = touching

        if math.dist(here, OFFLINE_GOAL_MM) <= OFFLINE_ARRIVED_MM:
            arrived = True
            break

        obstacles = obstacles_for(vision, skip_blue=0)
        if trigger == "always":
            due = True
        elif trigger == "periodic":
            due = (ticks_since_plan * CONTROL_TICK_S * 1000.0) % period_ms < (
                CONTROL_TICK_S * 1000.0)
        elif trigger == "emma":
            # Production reroute.evaluate_route(): direct-line check, then
            # target-moved / route-finished / active-waypoint-blocked /
            # periodic-safety-net -- the trigger actually shipping in
            # backbackup's VoronoiWaypointManager, not one of Paul's
            # experimental dynamic_scenario.py triggers.
            scene = PlanningScene(
                timestamp=elapsed,
                obstacles=tuple(
                    PlanningObstacle(robot_id=o.robot_id, isYellow=o.isYellow,
                                     pos_mm=o.pos_mm, radius_mm=o.radius_mm)
                    for o in obstacles
                ),
                field=FieldDimensions(),
            )
            due = evaluate_route(scene, here, OFFLINE_GOAL_MM, route_state).need_reroute
        else:
            due = path_blocked((here, *path), obstacles)

        if not path or due:
            previous = (here, *path) if path else ()
            new_path, ms = plan_for(planner, here, OFFLINE_GOAL_MM, obstacles,
                                    key=replans)
            stability.observe(previous, (here, *new_path) if new_path else ())
            path = new_path
            replans += 1
            plan_ms_total += ms
            ticks_since_plan = 0
            if trigger == "emma":
                commit_reroute(
                    route_state,
                    waypoints=tuple((p[0], p[1], 0.0) for p in new_path),
                    target_pose=(OFFLINE_GOAL_MM[0], OFFLINE_GOAL_MM[1], 0.0),
                )
        elif trigger == "emma":
            note_no_reroute(route_state)
        ticks_since_plan += 1

        aim = path[0] if path else OFFLINE_GOAL_MM
        if path and math.dist(here, aim) < ARRIVED_MM:
            path = path[1:]
            aim = path[0] if path else OFFLINE_GOAL_MM

        ex, ey = aim[0] - x, aim[1] - y
        dist = math.hypot(ex, ey)
        if dist > 1.0:
            scale = speed * min(1.0, dist / 400.0) / dist
            vx, vy = rotate_into_robot_frame(ex * scale, ey * scale, theta)
        else:
            vx = vy = 0.0
        dispatcher.publish(RobotCommand(robot_id=0, vx=vx, vy=vy, w=0.0, isYellow=False))

        elapsed_tick = time.perf_counter() - tick_start
        if elapsed_tick < CONTROL_TICK_S:
            time.sleep(CONTROL_TICK_S - elapsed_tick)

    dispatcher.publish(RobotCommand(robot_id=0, isYellow=False))

    return {
        "arrived": arrived,
        "navigation_s": time.time() - started,
        "accumulated_ms": plan_ms_total,
        "recalculated": replans,
        "collisions": collisions,
        "reversals": stability.reversals,
        "replans_compared": stability.replans_compared,
        "mean_heading_deg": stability.mean_heading_deg,
        "mean_shift_mm": stability.mean_shift_mm,
    }


def run_samples(dispatcher: RobotCommandDispatcher, vision: Vision, sender: grSimSender,
                 args, loops: tuple) -> list[dict]:
    """N discrete, resettable trials -- the live counterpart to
    dynamic_scenario.py's run_all_samples(): a fresh random obstacle phase
    per trial, drawn from the same seeded RNG pattern, so 200 trials are a
    distribution rather than 200 copies of one run."""
    rng = random.Random(args.seed)
    results = []
    for i in range(args.samples):
        phases = [rng.uniform(0, loop.length()) for loop in loops]
        r = run_trial(dispatcher, vision, sender, args.planner, args.trigger,
                     args.period_ms, args.speed, loops, args.obstacle_speed,
                     phases, args.max_s)
        if r is None:
            print(f"  sample {i + 1}/{args.samples}: setup failed (vision didn't "
                  f"confirm both teams after reset) -- skipped, not counted")
            continue
        results.append(r)
        print(f"  sample {i + 1}/{args.samples}: arrived={r['arrived']!s:5} "
              f"replans={r['recalculated']:3d} nav_s={r['navigation_s']:5.2f} "
              f"collisions={r['collisions']} reversals={r['reversals']}")
    return results


def report_samples(results: list[dict], args) -> None:
    if not results:
        print("\nNo completed trials -- nothing to report.")
        return
    arrived = sum(1 for r in results if r["arrived"])

    def stats(key):
        vals = [r[key] for r in results]
        return (statistics.mean(vals), statistics.pstdev(vals),
                min(vals), max(vals))

    print(f"\n=== live grSim, scenario {args.scenario}, {args.planner} planner, "
          f"{args.trigger} trigger ===")
    print(f"  n = {len(results)} trials ({args.samples} requested), obstacles "
          f"{args.obstacle_speed:.1f} m/s, robot {args.speed:.1f} m/s")
    print(f"  arrived {arrived}/{len(results)}")
    for label, key in (("replans", "recalculated"), ("nav s", "navigation_s"),
                        ("collisions", "collisions"), ("reversals", "reversals")):
        mean, sd, lo, hi = stats(key)
        print(f"  {label:<10} mean {mean:6.2f}  sd {sd:6.2f}  min {lo:6.2f}  max {hi:6.2f}")


STATIC_RUNNERS = {
    "prm": lambda req, seed: _run_prm(req, seed=seed, num_samples=40),
    "visibility": lambda req, seed: _run_visibility(req),
    "voronoi": lambda req, seed: _run_voronoi(req),
}

STATIC_SCENARIO_CHOICES = {
    "graph": STATIC_SCENARIOS[0],
    "sampling": STATIC_SCENARIOS[1],
    "mixed": STATIC_SCENARIOS[2],
    "stoppage": STATIC_SCENARIOS[3],
}


def place_static(sender: grSimSender, scenario) -> None:
    """Place the robot at the scenario's fixed start and every obstacle at
    its fixed, stationary position -- nothing patrols a loop here, that's
    the whole difference from the dynamic scenarios."""
    robots = [{
        "x": scenario.start_mm[0] / 1000.0, "y": scenario.start_mm[1] / 1000.0,
        "orientation": 0.0, "robot_id": 0, "isYellow": False,
    }]
    for i, obstacle in enumerate(scenario.obstacles):
        robots.append({
            "x": obstacle.pos_mm[0] / 1000.0, "y": obstacle.pos_mm[1] / 1000.0,
            "orientation": 0.0, "robot_id": i, "isYellow": True,
        })
    sender.send_packet(grSimPacketFactory.scenario_replacement_command(robots))


def run_static_trial(vision: Vision, planner: str, scenario, seed: int) -> dict:
    """One static call: read the live-detected obstacle positions (real
    vision noise, unlike static_scenario.py's ground truth) and plan once
    from the scenario's fixed start to its fixed goal. No movement, no
    trigger -- this isolates whether vision noise on obstacle position
    changes the outcome from the offline, ground-truth version."""
    vision.update()
    # obstacles_for() returns every robot currently on the pitch, but grSim's
    # configured team size (bumped to fit the largest scenario, 15 for
    # worst_case_sampling) is bigger than this scenario's own obstacle count
    # -- the extra robots are phantom obstacles left over from whatever ran
    # before, not part of this scenario, so they're excluded explicitly here
    # rather than trusting "everyone vision currently sees."
    # Radius comes from the scenario's own declared value, not a live-sensed
    # property -- game_stoppage's ball obstacle is a 500mm keep-out riding
    # on an ordinary 90mm robot in grSim, and vision has no way to report
    # "this one means more." Only position is live; radius is the scenario's.
    obstacles = tuple(
        Obstacle(pos_mm=vision.yellow[i][:2], radius_mm=scenario.obstacles[i].radius_mm,
                robot_id=i, isYellow=True)
        for i in range(len(scenario.obstacles))
        if i in vision.yellow
    )
    request = PlanRequest(start_mm=scenario.start_mm, goal_mm=scenario.goal_mm,
                          obstacles=obstacles)
    t0 = time.perf_counter()
    plan = STATIC_RUNNERS[planner](request, seed)
    ms = (time.perf_counter() - t0) * 1000.0
    if plan.success and len(plan.waypoints_mm) >= 2:
        return {
            "success": True, "time_ms": ms,
            "length_m": path_length_m(plan.waypoints_mm),
            "safety_m": path_safety_m(plan.waypoints_mm, obstacles),
        }
    return {"success": False, "time_ms": ms, "length_m": None, "safety_m": None}


def run_static_samples(vision: Vision, sender: grSimSender, planner: str,
                        scenario, samples: int) -> list[dict] | None:
    """Place once (nothing moves), then call the planner `samples` times,
    each with a fresh seed for PRM and a freshly re-read vision snapshot
    (so per-frame detection jitter is real, even though the true positions
    never change). Returns None if the reset couldn't be confirmed."""
    place_static(sender, scenario)
    time.sleep(0.3)
    if not vision.wait_for_robots(1, len(scenario.obstacles), timeout_s=8.0):
        return None
    return [run_static_trial(vision, planner, scenario, seed=i) for i in range(samples)]


def report_static_samples(results: list[dict], planner: str, scenario_key: str) -> None:
    if not results:
        print("\nNo completed trials -- nothing to report.")
        return
    successes = [r for r in results if r["success"]]
    failures = len(results) - len(successes)

    def stats(key):
        vals = [r[key] for r in successes]
        if not vals:
            return "--", "--", "--", "--"
        return (f"{statistics.mean(vals):.2f}", f"{statistics.pstdev(vals):.2f}",
                f"{min(vals):.2f}", f"{max(vals):.2f}")

    print(f"\n=== live grSim static, {scenario_key}, {planner} planner ===")
    print(f"  n = {len(results)}, failures = {failures}")
    for label, key in (("time_ms", "time_ms"), ("length_m", "length_m"), ("safety_m", "safety_m")):
        m, sd, lo, hi = stats(key)
        print(f"  {label:<10} mean {m}  sd {sd}  min {lo}  max {hi}")


def selftest() -> int:
    """Pin the parts that do not need a running simulator."""
    failures = []

    # A robot facing +y must be told to drive along its own -x to move world +x.
    vx, vy = rotate_into_robot_frame(1.0, 0.0, math.pi / 2)
    if abs(vx) > 1e-9 or abs(vy + 1.0) > 1e-9:
        failures.append(f"frame rotation wrong at 90 deg: got ({vx:.3f}, {vy:.3f})")
    vx, vy = rotate_into_robot_frame(1.0, 0.0, 0.0)
    if abs(vx - 1.0) > 1e-9 or abs(vy) > 1e-9:
        failures.append(f"frame rotation wrong at 0 deg: got ({vx:.3f}, {vy:.3f})")

    # The trigger has to fire on an obstacle sitting on the path and not on one
    # far away, or every run replans either always or never.
    path = ((0.0, 0.0), (2000.0, 0.0))
    near = (Obstacle(pos_mm=(1000.0, 150.0), radius_mm=90.0, robot_id=1, isYellow=True),)
    far = (Obstacle(pos_mm=(1000.0, 900.0), radius_mm=90.0, robot_id=1, isYellow=True),)
    if not path_blocked(path, near):
        failures.append("trigger missed an obstacle 150 mm off the path")
    if path_blocked(path, far):
        failures.append("trigger fired on an obstacle 900 mm off the path")

    # Every backend must return waypoints that do not start on the robot, so the
    # controller does not chase the point it is already standing on.
    obstacles = (Obstacle(pos_mm=(0.0, 0.0), radius_mm=90.0, robot_id=9, isYellow=True),)
    for name in PLANNERS:
        waypoints, ms = plan_for(name, (-2000.0, 0.0), (2000.0, 0.0), obstacles, key=("test", 0))
        if waypoints and math.dist(waypoints[0], (-2000.0, 0.0)) < 1.0:
            failures.append(f"{name}: first waypoint is the robot's own position")
        if ms <= 0.0:
            failures.append(f"{name}: planning time not measured")

    for line in failures:
        print(f"FAIL {line}")
    if not failures:
        print("drive_grsim selftest OK")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--planner", choices=sorted(PLANNERS), default="voronoi")
    parser.add_argument("--robots", type=int, default=1, help="Blue robots to drive")
    parser.add_argument("--scenario", choices=("1", "2"), default="1",
                        help="Obstacle layout from dynamic_scenario.py: "
                             "1 = seven obstacles, 2 = five")
    parser.add_argument("--obstacle-speed", type=float, default=1.0,
                        help="Obstacle speed, m/s (offline default is 1.0)")
    parser.add_argument("--speed", type=float, default=1.5, help="Blue robot speed, m/s")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--trigger", choices=("geometric", "periodic", "always", "emma"),
                        default="geometric",
                        help="geometric/periodic/always are dynamic_scenario.py's "
                             "trigger family; emma is production reroute.evaluate_route "
                             "(active-waypoint check + target-moved + periodic safety net)")
    parser.add_argument("--period-ms", type=float, default=500.0,
                        help="Replan interval for --trigger periodic")
    parser.add_argument("--no-place", action="store_true",
                        help="Leave the robots where they are")
    parser.add_argument("--samples", type=int, default=None,
                        help="Run N discrete, resettable trials (dynamic_scenario.py's "
                             "own methodology) instead of one continuous session")
    parser.add_argument("--seed", type=int, default=0,
                        help="RNG seed for per-trial obstacle phases, with --samples")
    parser.add_argument("--max-s", type=float, default=30.0,
                        help="Per-trial timeout, with --samples (offline default is 30s)")
    parser.add_argument("--out-json", metavar="PATH",
                        help="With --samples: save per-trial results to this file")
    parser.add_argument("--static", choices=sorted(STATIC_SCENARIO_CHOICES),
                        help="Run static_scenario.py's stationary-obstacle test instead of "
                             "a dynamic one: graph/sampling/mixed/stoppage. Uses --samples "
                             "(default 60) and --planner; ignores --scenario, --trigger, "
                             "--obstacle-speed, --speed, --duration.")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    vision = Vision()
    sender = grSimSender()
    dispatcher = RobotCommandDispatcher(sender.send_robot_command, send_hz=COMMAND_HZ)
    dispatcher.start()

    if args.static:
        scenario = STATIC_SCENARIO_CHOICES[args.static]
        samples = args.samples if args.samples is not None else 60
        print(f"\n{args.planner} planner, static scenario '{args.static}' "
              f"({len(scenario.obstacles)} obstacles, stationary)")
        print(f"Running {samples} calls, live-detected obstacle positions.\n")
        results = run_static_samples(vision, sender, args.planner, scenario, samples)
        if results is None:
            print(f"Only saw {len(vision.blue)} blue and {len(vision.yellow)} yellow robots.")
            print("Is grSim running, and is this process on the same side of WSL as it?")
            dispatcher.stop()
            return 1
        report_static_samples(results, args.planner, args.static)
        if args.out_json:
            Path(args.out_json).parent.mkdir(parents=True, exist_ok=True)
            with open(args.out_json, "w") as f:
                json.dump({
                    "static_scenario": args.static, "planner": args.planner,
                    "samples_requested": samples, "results": results,
                }, f, indent=2)
            print(f"\nSaved {len(results)} trial results to {args.out_json}")
        dispatcher.stop()
        return 0

    loops = SCENARIOS[args.scenario]
    obstacles_n = len(loops)

    if args.samples is not None:
        print(f"\n{args.planner} planner, scenario {args.scenario} ({obstacles_n} "
              f"obstacles at {args.obstacle_speed:.1f} m/s), {args.trigger} trigger, "
              f"robot {args.speed:.1f} m/s")
        print(f"Running {args.samples} discrete trials, seed {args.seed}, "
              f"{args.max_s:.0f}s timeout each.\n")
        results = run_samples(dispatcher, vision, sender, args, loops)
        report_samples(results, args)
        if args.out_json:
            Path(args.out_json).parent.mkdir(parents=True, exist_ok=True)
            with open(args.out_json, "w") as f:
                json.dump({
                    "scenario": args.scenario, "planner": args.planner,
                    "trigger": args.trigger, "obstacle_speed": args.obstacle_speed,
                    "robot_speed": args.speed, "seed": args.seed,
                    "samples_requested": args.samples, "results": results,
                }, f, indent=2)
            print(f"\nSaved {len(results)} trial results to {args.out_json}")
        dispatcher.stop()
        return 0

    if not args.no_place:
        place_teams(sender, args.robots, loops)
        time.sleep(0.5)

    if not vision.wait_for_robots(args.robots, obstacles_n):
        print(f"Only saw {len(vision.blue)} blue and {len(vision.yellow)} yellow robots.")
        print("Is grSim running, and is this process on the same side of WSL as it?")
        return 1

    lanes = [(i - (args.robots - 1) / 2.0) * LANE_SPACING_MM for i in range(args.robots)]
    robots = [Robot(robot_id=i, target=(GOAL_X, lanes[i])) for i in range(args.robots)]

    print(f"\n{args.planner} planner, {args.robots} robot(s) at {args.speed:.1f} m/s, "
          f"scenario {args.scenario} ({obstacles_n} obstacles at "
          f"{args.obstacle_speed:.1f} m/s), {args.trigger} trigger")
    print(f"Running {args.duration:.0f} s. Watch the pitch; the numbers are below.\n")

    started = time.time()
    last_report = 0.0
    ticks = 0

    try:
        while time.time() - started < args.duration:
            tick_start = time.perf_counter()
            vision.update()
            ticks += 1
            drive_obstacles(dispatcher, vision, loops, args.obstacle_speed,
                            time.time() - started)

            for robot in robots:
                pose = vision.blue.get(robot.robot_id)
                if pose is None:
                    continue
                x, y, theta = pose
                here = (x, y)

                if math.dist(here, robot.target) <= ARRIVED_MM:
                    robot.laps += 1
                    robot.target = (START_X if robot.target[0] > 0 else GOAL_X,
                                    robot.target[1])
                    robot.path = ()

                obstacles = obstacles_for(vision, robot.robot_id)
                robot.closest_mm = min(
                    robot.closest_mm,
                    min((math.dist(here, o.pos_mm) for o in obstacles), default=float("inf")),
                )

                if args.trigger == "always":
                    due = True
                elif args.trigger == "periodic":
                    due = (ticks * CONTROL_TICK_S * 1000.0) % args.period_ms < (
                        CONTROL_TICK_S * 1000.0)
                else:
                    due = path_blocked((here, *robot.path), obstacles)

                if not robot.path or due:
                    previous = (here, *robot.path) if robot.path else ()
                    new_path, ms = plan_for(args.planner, here, robot.target,
                                            obstacles, key=robot.replans)
                    robot.stability.observe(previous, (here, *new_path) if new_path else ())
                    robot.path = new_path
                    robot.replans += 1
                    robot.plan_ms_total += ms

                # Drive at the next waypoint, or straight at the target when the
                # planner returned nothing to follow.
                aim = robot.path[0] if robot.path else robot.target
                if robot.path and math.dist(here, aim) < ARRIVED_MM:
                    robot.path = robot.path[1:]
                    aim = robot.path[0] if robot.path else robot.target

                ex, ey = aim[0] - x, aim[1] - y
                dist = math.hypot(ex, ey)
                if dist > 1.0:
                    scale = args.speed * min(1.0, dist / 400.0) / dist
                    vx, vy = rotate_into_robot_frame(ex * scale, ey * scale, theta)
                else:
                    vx = vy = 0.0
                dispatcher.publish(
                    RobotCommand(robot_id=robot.robot_id, vx=vx, vy=vy, w=0.0, isYellow=False)
                )

            now = time.time() - started
            if now - last_report >= 2.0:
                last_report = now
                parts = []
                for r in robots:
                    s = r.stability
                    parts.append(
                        f"#{r.robot_id} laps {r.laps} replans {r.replans:3d} "
                        f"rev {s.reversals:3d} {r.plan_ms_total / max(r.replans, 1):5.1f}ms"
                    )
                closest = min(r.closest_mm for r in robots)
                print(f"  t={now:5.1f}s  " + "  ".join(parts) +
                      f"   closest {closest:.0f}mm")

            elapsed = time.perf_counter() - tick_start
            if elapsed < CONTROL_TICK_S:
                time.sleep(CONTROL_TICK_S - elapsed)

    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        for r in robots:
            dispatcher.publish(RobotCommand(robot_id=r.robot_id, isYellow=False))
        for i in range(obstacles_n):
            dispatcher.publish(RobotCommand(robot_id=i, isYellow=True))
        time.sleep(0.3)
        dispatcher.stop()

    print(f"\n{'robot':<8}{'laps':>6}{'replans':>9}{'reversals':>11}"
          f"{'mean ms':>9}{'closest mm':>12}")
    print("-" * 55)
    for r in robots:
        print(f"#{r.robot_id:<7}{r.laps:>6}{r.replans:>9}{r.stability.reversals:>11}"
              f"{r.plan_ms_total / max(r.replans, 1):>9.2f}{r.closest_mm:>12.0f}")
    print(f"\nClosest approach below {CONTACT_MM:.0f} mm means the robots touched.")
    print("Reversals are replans that turned the robot more than 90 degrees from")
    print("the direction it was already going; see scripts/path_stability.py.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
