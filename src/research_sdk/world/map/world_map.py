from math import hypot
import time
from typing import Iterable, Optional

from research_sdk.world.map.geometry import linear_velocity
from research_sdk.world.map.obstacles import Obstacle
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene
from research_sdk.config import (
    DEFENCE_X_MM,
    DEFENCE_Y_MM,
    FIELD_X_MAX,
    FIELD_X_MIN,
    FIELD_Y_MAX,
    FIELD_Y_MIN,
    GOAL_DEPTH_MM,
    GOAL_HALF_WIDTH_MM,
    ROBOT_RADIUS_MM,
    VORONOI_HORIZON_MS,
    VORONOI_OBSTACLE_COST_WEIGHT,
    VORONOI_RENDER_DENSITY_PERCENT,
    VORONOI_RENDER_MAX_DENSITY_NODES,
)


R = ROBOT_RADIUS_MM   # mm - robot radius
BALL_MIN_CONFIDENCE = 0.1
BALL_BASE_TOLERANCE_MM = 150.0
BALL_TOLERANCE_RATE_MMPS = 7000.0


class WorldMap:
    def __init__(self, horizon_ms=VORONOI_HORIZON_MS, field=None, snapshot=None) -> None:
        self.horizon_ms = horizon_ms
        self.field = None
        self.robots: dict[tuple[bool, int], Obstacle] = {}
        self.ball: tuple[float, float] | None = None
        self.ball_vel_mmps = (0.0, 0.0)
        self._ball_timestamp: float | None = None
        self._ball_received_at_s: float | None = None
        self.ball_visible = False
        self.ball_last_seen_s: float | None = None
        self.last_rejected_ball_pos_mm: tuple[float, float] | None = None
        self.last_ball_rejection_reason: str | None = None
        self.possible_ball_left_field_pos_mm: tuple[float, float] | None = None
        self.ball_left_field_pos_mm: tuple[float, float] | None = None
        self.obs: list[Obstacle] = []

        if field is not None:
            self._create_field(field)

        if snapshot is not None:
            self.update(snapshot)

    def update(self, snapshot, field=None, received_at_s=None) -> None:
        if field is not None:
            self._create_field(field)
        if snapshot is not None:
            if received_at_s is None:
                received_at_s = time.time()
            self._extract_from_snap(snapshot, received_at_s)

    # -------------------------
    # Map creation
    # -------------------------
    def _create_field(self, field) -> None:
        # SSL-Vision geometry packets are no longer applied to a live-update
        # layer; field dimensions are fixed to the static constants in
        # ssl_field_config.yaml (FIELD_LENGTH_MM, DEFENCE_X_MM, etc.).
        self.field = field

    def _extract_from_snap(self, snapshot, received_at_s: float) -> None:
        timestamp = float(snapshot.timestamp)
        robots = (
            robot
            for team in (snapshot.yellow, snapshot.blue)
            for robot in team
            if robot is not None
        )
        self.obs = self.robots2obs(robots, timestamp, received_at_s)
        self.robots = {
            (obs.isYellow, obs.robot_id): obs
            for obs in self.obs
        }
        ball_candidates = getattr(snapshot, "ball_candidates", ())
        if not ball_candidates and snapshot.ball is not None:
            ball_candidates = (snapshot.ball,)
        self._update_ball_candidates(ball_candidates, timestamp, received_at_s)
        if snapshot.ball_left_field is not None:
            self.ball_left_field_pos_mm = snapshot.ball_left_field

    def robots2obs(
        self,
        robots: Iterable,
        timestamp: float,
        received_at_s: float | None = None,
    ) -> list[Obstacle]:
        obstacles = []
        for robot in robots:
            obs = Obstacle(
                timestamp=timestamp,
                robot_id=robot.id,
                isYellow=robot.isYellow,
                pos_mm=robot.pose,
                received_at_s=received_at_s,
            )
            older_obs = self.robots.get((obs.isYellow, obs.robot_id))
            if older_obs is not None and older_obs.timestamp < obs.timestamp:
                obs.update_vel_from(older_obs)
            obstacles.append(obs)
        return obstacles

    def _update_ball_candidates(
        self,
        balls,
        timestamp: float,
        received_at_s: float,
    ) -> None:
        balls = tuple(balls)
        if not balls:
            self.ball_visible = False
            return

        confident = [
            ball
            for ball in balls
            if float(getattr(ball, "confidence", 1.0)) >= BALL_MIN_CONFIDENCE
        ]
        if not confident:
            self.last_rejected_ball_pos_mm = balls[0].position
            self.last_ball_rejection_reason = "low_confidence"
            self.ball_visible = False
            return

        in_field = [
            ball
            for ball in confident
            if self._is_ball_in_field(ball.position)
        ]
        if not in_field:
            position = max(
                confident,
                key=lambda ball: float(getattr(ball, "confidence", 1.0)),
            ).position
            self.possible_ball_left_field_pos_mm = position
            self.last_rejected_ball_pos_mm = position
            self.last_ball_rejection_reason = "out_of_bounds"
            self.ball_visible = False
            return

        dt_s = None
        predicted = None
        if (
            self.ball is not None
            and self._ball_timestamp is not None
            and self._ball_timestamp < timestamp
        ):
            dt_s = timestamp - self._ball_timestamp
            predicted = self._predict_ball(dt_s)
            allowed_error_mm = (
                BALL_BASE_TOLERANCE_MM
                + BALL_TOLERANCE_RATE_MMPS * dt_s
            )
            plausible = [
                ball
                for ball in in_field
                if self._distance_to_ball_prediction(ball.position, predicted)
                <= allowed_error_mm
            ]
            if not plausible:
                closest = min(
                    in_field,
                    key=lambda ball: self._distance_to_ball_prediction(
                        ball.position,
                        predicted,
                    ),
                )
                self.last_rejected_ball_pos_mm = closest.position
                self.last_ball_rejection_reason = "trajectory_error"
                self.ball_visible = False
                return
        else:
            plausible = in_field

        ball = max(
            plausible,
            key=lambda candidate: self._ball_candidate_rank(candidate, predicted),
        )
        position = ball.position
        if dt_s is None:
            self.ball_vel_mmps = (0.0, 0.0)
        else:
            self.ball_vel_mmps = (
                linear_velocity(self.ball[0], position[0], dt_s),
                linear_velocity(self.ball[1], position[1], dt_s),
            )
        self.ball = position
        self._ball_timestamp = timestamp
        self._ball_received_at_s = received_at_s
        self.ball_last_seen_s = timestamp
        self.ball_visible = True
        self.last_rejected_ball_pos_mm = None
        self.last_ball_rejection_reason = None

    def _ball_candidate_rank(self, ball, predicted) -> tuple[float, float]:
        confidence = float(getattr(ball, "confidence", 1.0))
        if predicted is None:
            return confidence, 0.0
        distance = self._distance_to_ball_prediction(ball.position, predicted)
        return confidence, -distance

    def _distance_to_ball_prediction(
        self,
        position: tuple[float, float],
        predicted: tuple[float, float],
    ) -> float:
        return hypot(
            position[0] - predicted[0],
            position[1] - predicted[1],
        )

    def _predict_ball(self, dt_s: float) -> tuple[float, float]:
        return (
            self.ball[0] + self.ball_vel_mmps[0] * dt_s,
            self.ball[1] + self.ball_vel_mmps[1] * dt_s,
        )

    def _is_ball_in_field(self, position: tuple[float, float]) -> bool:
        if self.field is None:
            return (
                FIELD_X_MIN <= position[0] <= FIELD_X_MAX
                and FIELD_Y_MIN <= position[1] <= FIELD_Y_MAX
            )
        half_length = self.field.field_length / 2.0
        half_width = self.field.field_width / 2.0
        return (
            -half_length <= position[0] <= half_length
            and -half_width <= position[1] <= half_width
        )

    def ball_age_s(self, now_s: float | None = None) -> float | None:
        """Return seconds since the latest accepted ball observation."""
        if self.ball_last_seen_s is None:
            return None
        if now_s is None:
            now_s = time.time()
        reference_s = (
            self._ball_received_at_s
            if self._ball_received_at_s is not None
            else self.ball_last_seen_s
        )
        return max(0.0, now_s - reference_s)

    # -------------------------
    # Render/debug
    # -------------------------
    def get_render_data(
        self,
        now_s=None,
        horizon_ms=VORONOI_HORIZON_MS,
        extra_layers=(),
        include_voronoi=False,
        voronoi_density_percent=VORONOI_RENDER_DENSITY_PERCENT,
        voronoi_max_density_nodes=VORONOI_RENDER_MAX_DENSITY_NODES,
        voronoi_obstacle_cost_weight=VORONOI_OBSTACLE_COST_WEIGHT,
    ):
        """Return serializable, toggleable layers for a debug renderer."""
        from research_sdk.ui.renderer import Renderer

        extra_layers = tuple(extra_layers)
        if include_voronoi:
            from research_sdk.ui.voronoi_overlay import build_voronoi_overlay

            overlay = build_voronoi_overlay(
                self,
                now_s=now_s,
                horizon_ms=horizon_ms,
                density_percent=voronoi_density_percent,
                max_density_nodes=voronoi_max_density_nodes,
                obstacle_cost_weight=voronoi_obstacle_cost_weight,
            )
            self.last_voronoi_generation_ms = overlay.generation_ms
            extra_layers = (*extra_layers, overlay.layer)

        return Renderer(prediction_horizon_ms=horizon_ms).render(
            self,
            now_s=now_s,
            extra_layers=extra_layers,
        )

    # -------------------------
    # Trajectory queries
    # -------------------------
    def get_robot_trajectory(
        self,
        robot_id: int,
        isYellow: bool,
        horizon_ms: int | float | None = None,
    ) -> tuple[tuple[float, float], tuple[float, float]] | None:
        obs = self._get_robot_obstacle(robot_id, isYellow)
        if obs is None:
            return None
        if horizon_ms is None:
            horizon_ms = self.horizon_ms
        return obs.predicted_pos(horizon_ms), obs.vel_mmps

    def get_ball_trajectory(
        self,
        horizon_ms: int | float | None = None,
    ) -> tuple[tuple[float, float], tuple[float, float]] | None:
        if self.ball is None:
            return None
        if horizon_ms is None:
            horizon_ms = self.horizon_ms
        if horizon_ms < 0:
            raise ValueError("horizon_ms must be non-negative")
        dt_s = horizon_ms / 1000.0
        return (
            (
                self.ball[0] + self.ball_vel_mmps[0] * dt_s,
                self.ball[1] + self.ball_vel_mmps[1] * dt_s,
            ),
            self.ball_vel_mmps,
        )

    # -------------------------
    # Obstacle queries
    # -------------------------
    def get_obstacles(self) -> list[Obstacle]:
        return self.obs

    def get_planning_obstacles(
        self,
        now_s: float | None = None,
        horizon_ms: int | float | None = None,
        ignore_robots: set[tuple[bool, int]] | None = None,
    ) -> tuple[PlanningObstacle, ...]:
        """Return an immutable, age-adjusted obstacle view for path planning."""
        if now_s is None:
            now_s = time.time()
        if horizon_ms is None:
            horizon_ms = self.horizon_ms
        if horizon_ms < 0:
            raise ValueError("horizon_ms must be non-negative")
        if ignore_robots is None:
            ignore_robots = set()

        x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
        planning_obstacles = []
        for obs in self.obs:
            if (obs.isYellow, obs.robot_id) in ignore_robots:
                continue
            px, py = obs.pos_mm[0], obs.pos_mm[1]
            if not (x_min <= px <= x_max and y_min <= py <= y_max):
                continue
            age_ms = obs.age_s(now_s) * 1000.0
            prediction_horizon_ms = age_ms + horizon_ms
            planning_obstacles.append(
                PlanningObstacle(
                    robot_id=obs.robot_id,
                    isYellow=obs.isYellow,
                    pos_mm=obs.predicted_pos(prediction_horizon_ms),
                    radius_mm=obs.dynamic_radius_0(prediction_horizon_ms),
                    vel_mmps=obs.vel_mmps,
                    observation_age_ms=age_ms,
                    prediction_horizon_ms=prediction_horizon_ms,
                )
            )
        return tuple(planning_obstacles)

    def planning_scene(
        self,
        *,
        now_s: float | None = None,
        horizon_ms: int | float | None = None,
    ) -> PlanningScene:
        """Freeze tracked state into the common planner input boundary."""
        if now_s is None:
            now_s = time.time()
        if horizon_ms is None:
            horizon_ms = self.horizon_ms
        field = FieldDimensions()
        if self.field is not None:
            length = float(getattr(self.field, "field_length", field.field_length))
            width = float(getattr(self.field, "field_width", field.field_width))
            if length > 0.0 and width > 0.0:
                field = FieldDimensions(length, width)
        return PlanningScene(
            timestamp=now_s,
            obstacles=self.get_planning_obstacles(now_s=now_s, horizon_ms=horizon_ms),
            field=field,
            prediction_horizon_ms=float(horizon_ms),
        )

    def find_closest_robot_to(
        self,
        start_pos: tuple[float, float],
        isYellow: Optional[bool] = None,
    ) -> tuple[int, tuple[float, float, float]] | None:
        """Locate the closest robot to start_pos, optionally filtered by team."""
        closest = None
        closest_dist = float("inf")

        for obs in self.obs:
            if isYellow is not None and obs.isYellow != isYellow:
                continue
            dist = obs.dist_to(start_pos)
            if dist < closest_dist:
                closest_dist = dist
                closest = obs

        if closest is None:
            return None
        return closest.robot_id, closest.pos_mm

    def get_nearby_teammates(
        self,
        robot_id: int,
        isYellow: bool,
        radius_mm: float = 500,
    ) -> list[int]:
        """Return sorted teammate robot IDs within radius_mm."""
        robot = self._get_robot_obstacle(robot_id, isYellow)
        if robot is None:
            return []

        nearby = []
        for obs in self.obs:
            if obs.robot_id == robot_id and obs.isYellow == isYellow:
                continue
            if obs.isYellow != isYellow:
                continue
            dist = robot.dist_to(obs.pos_mm[:2])
            if dist <= radius_mm:
                nearby.append((dist, obs.robot_id))

        nearby.sort(key=lambda item: item[0])
        return [nearby_robot_id for _, nearby_robot_id in nearby]

    def get_nearby_enemies(
        self,
        robot_id: int,
        isYellow: bool,
        radius_mm: float = 500,
    ) -> list[int]:
        """Return sorted enemy robot IDs within radius_mm."""
        robot = self._get_robot_obstacle(robot_id, isYellow)
        if robot is None:
            return []

        nearby = []
        for obs in self.obs:
            if obs.isYellow == isYellow:
                continue
            dist = robot.dist_to(obs.pos_mm[:2])
            if dist <= radius_mm:
                nearby.append((dist, obs.robot_id))

        nearby.sort(key=lambda item: item[0])
        return [nearby_robot_id for _, nearby_robot_id in nearby]

    def _get_robot_obstacle(
        self,
        robot_id: int,
        isYellow: bool,
    ) -> Obstacle | None:
        return self.robots.get((isYellow, robot_id))

    def is_path_free(
        self,
        start_pos: tuple[float, float],
        end_pos: tuple[float, float],
        ignore_robots: set[tuple[bool, int]] | None = None,
        clearance: float = 0.0,
        horizon_ms: int | float | None = None,
    ) -> bool:
        if ignore_robots is None:
            ignore_robots = set()
        if horizon_ms is None:
            horizon_ms = self.horizon_ms

        for obs in self.obs:
            if (obs.isYellow, obs.robot_id) in ignore_robots:
                continue
            if obs.intersects_path(
                start_pos_mm=start_pos,
                end_pos_mm=end_pos,
                extra_clearance_mm=clearance,
                horizon_ms=horizon_ms,
            ):
                return False
        return True

    def is_target_in_box(
        self,
        target_pos: tuple[float, float],
        x_lim: int | float,
        y_lim: int | float,
        offset=R,
    ) -> bool:
        """Return whether target_pos is inside centered limits with an inset."""
        x, y = target_pos
        return abs(x) <= x_lim - offset and abs(y) <= y_lim - offset
