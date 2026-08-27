"""Serializable render primitives for visualizing the tracked world map."""

from dataclasses import dataclass
from typing import Iterable, TYPE_CHECKING

from research_sdk.config import BALL_R, VORONOI_HORIZON_MS

if TYPE_CHECKING:
    from research_sdk.world.map.world_map import WorldMap


YELLOW = "#ffd700"
BLUE = "#4a90d9"
BALL = "#ff8c00"
PREDICTION = "#e94560"
VELOCITY = "#2ecc71"


@dataclass(frozen=True, slots=True)
class RenderCircle:
    center_mm: tuple[float, float]
    radius_mm: float
    color: str
    label: str = ""
    filled: bool = False


@dataclass(frozen=True, slots=True)
class RenderVector:
    start_mm: tuple[float, float]
    end_mm: tuple[float, float]
    color: str
    label: str = ""


@dataclass(frozen=True, slots=True)
class RenderRobot:
    center_mm: tuple[float, float]
    orientation_rad: float
    color: str
    label: str = ""


@dataclass(frozen=True, slots=True)
class RenderPolyline:
    points_mm: tuple[tuple[float, float], ...]
    color: str
    closed: bool = False


@dataclass(frozen=True, slots=True)
class RenderLayer:
    """A toggleable group of primitives.

    Future map generators can append layers such as Voronoi edges without
    importing Qt or changing the canvas.
    """

    name: str
    robots: tuple[RenderRobot, ...] = ()
    circles: tuple[RenderCircle, ...] = ()
    vectors: tuple[RenderVector, ...] = ()
    polylines: tuple[RenderPolyline, ...] = ()
    visible_by_default: bool = True


@dataclass(frozen=True, slots=True)
class MapRenderData:
    """Serializable render frame passed from the world model to the GUI."""

    layers: tuple[RenderLayer, ...]

    def layer(self, name: str) -> RenderLayer | None:
        return next((layer for layer in self.layers if layer.name == name), None)


class Renderer:
    """Build toggleable debug layers from a tracked :class:`WorldMap`."""

    def __init__(
        self,
        prediction_horizon_ms: int | float = VORONOI_HORIZON_MS,
        velocity_vector_seconds: float = 0.25,
    ) -> None:
        if prediction_horizon_ms < 0:
            raise ValueError("prediction_horizon_ms must be non-negative")
        if velocity_vector_seconds < 0:
            raise ValueError("velocity_vector_seconds must be non-negative")
        self.prediction_horizon_ms = prediction_horizon_ms
        self.velocity_vector_seconds = velocity_vector_seconds

    def render(
        self,
        world_map: "WorldMap",
        now_s: float | None = None,
        extra_layers: Iterable[RenderLayer] = (),
    ) -> MapRenderData:
        robots = []
        velocity_vectors = []
        for obs in world_map.get_obstacles():
            color = YELLOW if obs.isYellow else BLUE
            center = (obs.pos_mm[0], obs.pos_mm[1])
            robots.append(
                RenderRobot(
                    center_mm=center,
                    orientation_rad=obs.pos_mm[2],
                    color=color,
                    label=str(obs.robot_id),
                )
            )
            velocity_vectors.append(
                RenderVector(
                    start_mm=center,
                    end_mm=(
                        center[0] + obs.vel_mmps[0] * self.velocity_vector_seconds,
                        center[1] + obs.vel_mmps[1] * self.velocity_vector_seconds,
                    ),
                    color=VELOCITY,
                    label=f"{obs.speed_mmps:.0f} mm/s",
                )
            )

        predicted_circles = tuple(
            RenderCircle(
                center_mm=obs.pos_mm,
                radius_mm=obs.radius_mm,
                color=PREDICTION,
                label=f"{obs.robot_id}",
            )
            for obs in world_map.get_planning_obstacles(
                now_s=now_s,
                horizon_ms=self.prediction_horizon_ms,
            )
        )

        ball_circles = ()
        ball_vectors = ()
        if world_map.ball is not None:
            ball_color = BALL if world_map.ball_visible else "#a86320"
            ball_circles = (
                RenderCircle(world_map.ball, BALL_R, ball_color, filled=True),
            )
            ball_vectors = (
                RenderVector(
                    start_mm=world_map.ball,
                    end_mm=(
                        world_map.ball[0]
                        + world_map.ball_vel_mmps[0] * self.velocity_vector_seconds,
                        world_map.ball[1]
                        + world_map.ball_vel_mmps[1] * self.velocity_vector_seconds,
                    ),
                    color=ball_color,
                    label=f"{world_map.ball_vel_mmps}",
                ),
            )

        layers = (
            RenderLayer("Robots", robots=tuple(robots)),
            RenderLayer("Velocity vectors", vectors=tuple(velocity_vectors)),
            RenderLayer(
                "Predicted clearance",
                circles=predicted_circles,
                visible_by_default=False,
            ),
            RenderLayer("Ball", circles=ball_circles, vectors=ball_vectors),
            *tuple(extra_layers),
        )
        return MapRenderData(layers=layers)
