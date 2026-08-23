"""Probabilistic Roadmap (PRM) + Dijkstra path planner.

This is a rewritten, SSL-correct version of the planner recovered from
TurtleRabbit's git history (`pathToPoint.py`, deleted, recovered from
`2025-TeamControl`'s full history) and its cited upstream source,
https://github.com/KaleabTessera/PRM-Path-Planning (per TurtleRabbit's own
published TDP, arXiv:2402.08205: "The path planner used in this study
implements a Probabilistic Roadmap (PRM) approach ... adapted from the open
source repository").

What changed vs. the original ``PRMController``, and why -- see
`docs/algorithms.md` for the full writeup, this is the short version:

1. **Real field boundaries.** The original hardcoded a 100x100-unit square
   (``genCoords(self, maxSizeOfMap=100)``) with no way to inject the actual
   9000x6000mm SSL field. Samples are now drawn from the request's
   ``field_length_mm``/``field_width_mm``.
2. **Circular obstacles, not axis-aligned rectangles.** The original's
   ``checkCollision``/``checkLineCollision`` assumed rectangular obstacles
   with ``bottomLeft``/``topLeft``/``bottomRight`` corners -- wrong for SSL,
   where every obstacle is a robot (a circle). Collision checks here are
   point-to-circle and segment-to-circle distance tests instead.
3. **No matplotlib in the hot path.** The original called ``plt.scatter``/
   ``plt.plot`` inside the sampling and neighbour-linking loops and blocked
   on ``plt.show()`` at the end of ``runPRM`` -- fine for a one-off teaching
   demo, unusable inside a 16ms planning cycle. This version does no
   plotting; ``scripts/demo_planners.py`` handles visualisation separately,
   from the returned waypoints.
4. **Dijkstra via networkx** instead of the original's hand-rolled
   ``Graph``/``dijkstra`` (``classes/Dijkstra.py``) -- same algorithm,
   fewer lines to maintain, and it's the same library TurtleRabbit's own
   Voronoi planner already uses (``nx.dijkstra_path``), so this stays
   consistent with the rest of the codebase's conventions.
"""

from __future__ import annotations

import math
import time

import networkx as nx
import numpy as np

from research_sdk.config import ROBOT_RADIUS_MM
from research_sdk.planners.common import Obstacle, PlanRequest, PlanResult, StepRecorder, path_length_mm
from research_sdk.planners.Dijkstra.waypoint_manager import PlannerInput, PlannerOutput


def _segment_circle_collision(
    p0: np.ndarray, p1: np.ndarray, centre: np.ndarray, radius: float
) -> bool:
    """True if the closed segment p0->p1 passes within `radius` of `centre`."""
    seg = p1 - p0
    seg_len_sq = float(seg @ seg)
    if seg_len_sq < 1e-9:
        return bool(np.linalg.norm(p0 - centre) <= radius)
    t = float((centre - p0) @ seg) / seg_len_sq
    t = max(0.0, min(1.0, t))
    closest = p0 + t * seg
    return bool(np.linalg.norm(closest - centre) <= radius)


def _point_in_any_obstacle(point: np.ndarray, obstacle_centres: np.ndarray, radii: np.ndarray) -> bool:
    if obstacle_centres.size == 0:
        return False
    dists = np.linalg.norm(obstacle_centres - point, axis=1)
    return bool(np.any(dists <= radii))


def _segment_free(
    p0: np.ndarray,
    p1: np.ndarray,
    obstacle_centres: np.ndarray,
    radii: np.ndarray,
) -> bool:
    for centre, radius in zip(obstacle_centres, radii):
        if _segment_circle_collision(p0, p1, centre, radius):
            return False
    return True


def plan(
    request: PlanRequest,
    *,
    num_samples: int = 20,
    k_neighbours: int = 6,
    max_resample_attempts: int = 5,
    seed: int | None = 0,
    record: StepRecorder | None = None,
) -> PlanResult:
    """Plan a path with PRM (random milestones + k-NN links) + Dijkstra.

    Mirrors TurtleRabbit's recovered calling convention: try a direct
    straight line from start to goal first (skips PRM entirely when the
    field is clear, same as the recovered ``pathToPoint.py``'s
    ``checkLineCollision`` short-circuit), and only fall back to sampling
    when that's blocked.

    ``num_samples=20`` matches the value in TurtleRabbit's recovered
    calling code (``sample = 20``); their TDP separately reports ``n=10``
    as their "effective compromise" for path diversity vs. compute cost --
    left as a tunable here rather than silently picking one, since the two
    sources disagree and neither documents exactly which config shipped.

    ``record``, if given a :class:`~research_sdk.planners.common.StepRecorder`,
    gets a log of every sample drawn and every candidate edge tested (see
    ``algorithms.viz.animate_construction``). Leave it ``None`` (the
    default) for normal/timed planning calls -- logging is skipped entirely
    when there's no recorder, so this costs nothing on the hot path.
    """
    start_t = time.perf_counter()

    start = np.array(request.start_mm, dtype=float)
    goal = np.array(request.goal_mm, dtype=float)
    total_clearance = request.total_clearance_mm

    obstacle_centres = np.array(
        [obs.pos_mm for obs in request.obstacles], dtype=float
    ).reshape(-1, 2)
    # Inflate each obstacle by (its own radius + clearance). total_clearance
    # already folds in the *planning robot's* radius, so subtract it back out
    # here to avoid double-counting it against the obstacle's own size.
    radii = (
        np.array(
            [obs.radius_mm + (total_clearance - request.robot_radius_mm) for obs in request.obstacles],
            dtype=float,
        )
        if request.obstacles
        else np.zeros(0)
    )

    if record is not None:
        record.log(
            "obstacles",
            start=tuple(start),
            goal=tuple(goal),
            centres=[tuple(c) for c in obstacle_centres],
            radii=list(radii),
            field_length_mm=request.field_length_mm,
            field_width_mm=request.field_width_mm,
        )

    if _point_in_any_obstacle(start, obstacle_centres, radii) or _point_in_any_obstacle(
        goal, obstacle_centres, radii
    ):
        return PlanResult(
            success=False,
            waypoints_mm=(),
            path_length_mm=0.0,
            planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
            message="start or goal lies inside an inflated obstacle",
        )

    # Direct line-of-sight shortcut -- same optimisation as the recovered
    # TurtleRabbit calling code (skip PRM when the straight line is clear).
    if _segment_free(start, goal, obstacle_centres, radii):
        waypoints = (tuple(start), tuple(goal))
        if record is not None:
            record.log("path", waypoints=waypoints, direct=True)
        return PlanResult(
            success=True,
            waypoints_mm=waypoints,
            path_length_mm=path_length_mm(waypoints),
            planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
            nodes_expanded=2,
            message="direct line of sight, PRM sampling skipped",
        )

    rng = np.random.default_rng(seed)
    x_half = request.field_length_mm / 2.0
    y_half = request.field_width_mm / 2.0

    for attempt in range(max_resample_attempts):
        samples = rng.uniform(
            low=[-x_half, -y_half], high=[x_half, y_half], size=(num_samples, 2)
        )
        nodes = np.vstack([samples, start.reshape(1, 2), goal.reshape(1, 2)])
        start_idx = num_samples
        goal_idx = num_samples + 1

        free_mask = np.array(
            [not _point_in_any_obstacle(p, obstacle_centres, radii) for p in nodes]
        )
        free_indices = np.where(free_mask)[0]

        if record is not None:
            for i, p in enumerate(samples):
                record.log("sample", point=tuple(p), free=bool(free_mask[i]), attempt=attempt)
        if start_idx not in free_indices or goal_idx not in free_indices:
            # Shouldn't happen (checked above) but guard against edge cases
            # where clearance math differs by a hair.
            continue

        graph = nx.Graph()
        graph.add_nodes_from(free_indices.tolist())

        free_points = nodes[free_indices]
        k = min(k_neighbours + 1, len(free_points))  # +1: a point is its own nearest neighbour
        if k < 2:
            continue

        # Brute-force k-NN (fine at these sample counts; avoids adding
        # scikit-learn as a dependency for ~20-30 points).
        diffs = free_points[:, None, :] - free_points[None, :, :]
        dist_matrix = np.linalg.norm(diffs, axis=2)
        neighbour_order = np.argsort(dist_matrix, axis=1)

        for local_i in range(len(free_points)):
            global_i = int(free_indices[local_i])
            for local_j in neighbour_order[local_i, 1:k]:
                global_j = int(free_indices[local_j])
                if graph.has_edge(global_i, global_j):
                    continue
                accepted = _segment_free(nodes[global_i], nodes[global_j], obstacle_centres, radii)
                if record is not None:
                    record.log(
                        "edge_test",
                        a=tuple(nodes[global_i]),
                        b=tuple(nodes[global_j]),
                        accepted=accepted,
                        attempt=attempt,
                    )
                if accepted:
                    weight = float(dist_matrix[local_i, local_j])
                    graph.add_edge(global_i, global_j, weight=weight)

        try:
            node_path = nx.dijkstra_path(graph, start_idx, goal_idx, weight="weight")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            seed = None if seed is None else seed + 1
            rng = np.random.default_rng(seed)
            continue

        waypoints = tuple(tuple(nodes[i]) for i in node_path)
        if record is not None:
            record.log("path", waypoints=waypoints, direct=False, attempt=attempt)
        return PlanResult(
            success=True,
            waypoints_mm=waypoints,
            path_length_mm=path_length_mm(waypoints),
            planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
            nodes_expanded=len(free_indices),
            message=f"PRM solved on attempt {attempt + 1}/{max_resample_attempts}",
        )

    return PlanResult(
        success=False,
        waypoints_mm=(),
        path_length_mm=0.0,
        planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
        message=f"no path found after {max_resample_attempts} resampling attempts",
    )


class PRMPlanner:
    """Adapts :func:`plan` above to the ``PlannerAPI.plan`` contract used by
    the UI's planner dropdown (see ``planners/api.py`` / ``ui/runtime.py``).
    """

    def __init__(self, **plan_kwargs) -> None:
        self._plan_kwargs = plan_kwargs

    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        request = _plan_request_from_planner_input(planner_input)
        result = plan(request, **self._plan_kwargs)
        return _planner_output_from_plan_result(planner_input, result)

    def reset(self, robot_id: int | None = None, is_yellow: bool | None = None) -> None:
        """Stateless planner -- nothing to clear, kept for interface parity."""


def _plan_request_from_planner_input(planner_input: PlannerInput) -> PlanRequest:
    start = (float(planner_input.current_pose[0]), float(planner_input.current_pose[1]))
    goal = (float(planner_input.target_pose[0]), float(planner_input.target_pose[1]))
    scene_obstacles = (
        planner_input.scene.get_planning_obstacles()
        if planner_input.scene is not None
        else planner_input.obstacles
    )
    obstacles = tuple(
        Obstacle(
            pos_mm=(float(o.pos_mm[0]), float(o.pos_mm[1])),
            radius_mm=float(o.radius_mm),
            robot_id=int(o.robot_id),
            isYellow=bool(o.isYellow),
        )
        for o in scene_obstacles
    )
    return PlanRequest(
        start_mm=start,
        goal_mm=goal,
        obstacles=obstacles,
        robot_radius_mm=ROBOT_RADIUS_MM,
        clearance_mm=planner_input.clearance_mm,
    )


def _planner_output_from_plan_result(planner_input: PlannerInput, result: PlanResult) -> PlannerOutput:
    target_pose = planner_input.target_pose
    heading = float(target_pose[2]) if len(target_pose) > 2 else 0.0
    waypoints = tuple((float(x), float(y), heading) for x, y in result.waypoints_mm)
    active_target_pose = waypoints[-1] if waypoints else (
        float(target_pose[0]),
        float(target_pose[1]),
        heading,
    )
    return PlannerOutput(
        waypoints=waypoints,
        current_waypoint_index=0,
        active_target_pose=active_target_pose,
        is_path_free=result.success and not waypoints,
        need_reroute=False,
        did_reroute=True,
    )
