"""Matplotlib helpers for inspecting generated Voronoi maps."""

from __future__ import annotations

from math import degrees
from pathlib import Path

from research_sdk.config import (
    DEFENCE_X_MM,
    DEFENCE_Y_MM,
    GOAL_DEPTH_MM,
    GOAL_WIDTH_MM,
)
from research_sdk.world.map.voronoi.voronoi_generator import BoundedVoronoiMap


def save_voronoi_map_plot(
    voronoi_map: BoundedVoronoiMap,
    output_path: str | Path,
    *,
    field_geometry=None,
    show: bool = False,
) -> Path:
    """Save a visual debug plot of a bounded Voronoi map over field lines."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(9, 6))
    x_min, x_max, y_min, y_max = voronoi_map.bounds_mm
    field_style = _draw_field(ax, voronoi_map, field_geometry)

    ax.set_title(
        f"Voronoi map: {len(voronoi_map.virtual_sites_mm)} virtual sites, "
        f"{len(voronoi_map.obstacles)} obstacles, "
        f"{len(voronoi_map.edges)} safe edges"
    )
    ax.set_xlabel("x mm")
    ax.set_ylabel("y mm")
    ax.set_xlim(*field_style["xlim"])
    ax.set_ylim(*field_style["ylim"])
    ax.set_aspect("equal", adjustable="box")
    ax.set_facecolor(field_style["field_color"])

    for cell in voronoi_map.cells:
        patch = Polygon(
            cell.polygon_mm,
            closed=True,
            fill=False,
            edgecolor="#64b5f6",
            linewidth=1.25,
            alpha=0.95,
            zorder=3,
        )
        ax.add_patch(patch)

    if voronoi_map.virtual_sites_mm:
        xs, ys = zip(*voronoi_map.virtual_sites_mm)
        ax.scatter(xs, ys, c="#e94560", s=18, label="virtual sites", zorder=5)

    node_by_id = {node.id: node for node in voronoi_map.nodes}
    for edge in voronoi_map.edges:
        start = node_by_id[edge.start_id]
        end = node_by_id[edge.end_id]
        ax.plot(
            [start.x, end.x],
            [start.y, end.y],
            color="#f2c94c",
            linewidth=2.0,
            alpha=0.95,
            zorder=4,
        )

    for obstacle in voronoi_map.obstacles:
        keepout_radius = obstacle.radius_mm + voronoi_map.min_clearance_mm
        ax.add_patch(
            plt.Circle(
                obstacle.pos_mm,
                keepout_radius,
                fill=True,
                facecolor="#8e7cc3",
                edgecolor="#f4dcff",
                linewidth=1.1,
                alpha=0.45,
                zorder=5,
            )
        )
        ax.add_patch(
            plt.Circle(
                obstacle.pos_mm,
                obstacle.radius_mm,
                fill=False,
                edgecolor="#3f1f5f",
                linewidth=1.0,
                alpha=0.85,
                zorder=6,
            )
        )
        ax.scatter(
            [obstacle.pos_mm[0]],
            [obstacle.pos_mm[1]],
            c="#6a329f",
            s=35,
            label="obstacles" if obstacle == voronoi_map.obstacles[0] else None,
            zorder=7,
        )
        if obstacle.label:
            ax.text(
                obstacle.pos_mm[0],
                obstacle.pos_mm[1],
                obstacle.label,
                color="#ffffff",
                fontsize=7,
                ha="center",
                va="center",
                zorder=8,
            )

    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)

    if show:
        plt.show()
    else:
        plt.close(fig)

    return output_path


def _draw_field(ax, voronoi_map: BoundedVoronoiMap, field_geometry):
    from matplotlib.patches import Circle, Rectangle

    field_color = "#1f7a3f"
    line_color = "#f3f7f2"
    goal_color = "#d6d6d6"

    field_x_min, field_x_max, field_y_min, field_y_max = getattr(
        voronoi_map,
        "field_bounds_mm",
        voronoi_map.bounds_mm,
    )
    nav_x_min, nav_x_max, nav_y_min, nav_y_max = voronoi_map.bounds_mm
    field_length = field_x_max - field_x_min
    field_width = field_y_max - field_y_min
    half_length = field_length / 2.0
    margin = _field_value(field_geometry, "boundary_width", 300.0)
    goal_depth = _field_value(field_geometry, "goal_depth", GOAL_DEPTH_MM)
    goal_width = _field_value(field_geometry, "goal_width", GOAL_WIDTH_MM)

    ax.add_patch(
        Rectangle(
            (field_x_min - margin - goal_depth, field_y_min - margin),
            field_length + 2.0 * margin + 2.0 * goal_depth,
            field_width + 2.0 * margin,
            facecolor=field_color,
            edgecolor="none",
            zorder=0,
        )
    )

    if field_geometry is not None and (
        getattr(field_geometry, "field_lines", None)
        or getattr(field_geometry, "field_arcs", None)
    ):
        _draw_ssl_vision_geometry(ax, field_geometry, line_color)
    else:
        ax.add_patch(
            Rectangle(
                (field_x_min, field_y_min),
                field_length,
                field_width,
                fill=False,
                edgecolor=line_color,
                linewidth=1.8,
                zorder=1,
            )
        )
        ax.plot(
            [0.0, 0.0],
            [field_y_min, field_y_max],
            color=line_color,
            linewidth=1.4,
            zorder=1,
        )
        ax.add_patch(
            Circle(
                (0.0, 0.0),
                500.0,
                fill=False,
                edgecolor=line_color,
                linewidth=1.4,
                zorder=1,
            )
        )
        ax.add_patch(Circle((0.0, 0.0), 25.0, color=line_color, zorder=1))

        penalty_depth = _field_value(field_geometry, "penalty_area_depth", DEFENCE_X_MM)
        penalty_width = _field_value(field_geometry, "penalty_area_width", DEFENCE_Y_MM)
        penalty_half_width = penalty_width / 2.0
        ax.add_patch(
            Rectangle(
                (-half_length, -penalty_half_width),
                penalty_depth,
                penalty_width,
                fill=False,
                edgecolor=line_color,
                linewidth=1.4,
                zorder=1,
            )
        )
        ax.add_patch(
            Rectangle(
                (half_length - penalty_depth, -penalty_half_width),
                penalty_depth,
                penalty_width,
                fill=False,
                edgecolor=line_color,
                linewidth=1.4,
                zorder=1,
            )
        )

    goal_half_width = goal_width / 2.0
    ax.add_patch(
        Rectangle(
            (field_x_min - goal_depth, -goal_half_width),
            goal_depth,
            goal_width,
            fill=False,
            edgecolor=goal_color,
            linewidth=1.4,
            zorder=1,
        )
    )
    ax.add_patch(
        Rectangle(
            (field_x_max, -goal_half_width),
            goal_depth,
            goal_width,
            fill=False,
            edgecolor=goal_color,
            linewidth=1.4,
            zorder=1,
        )
    )
    ax.add_patch(
        Rectangle(
            (nav_x_min, nav_y_min),
            nav_x_max - nav_x_min,
            nav_y_max - nav_y_min,
            fill=False,
            edgecolor="#ffdf6e",
            linewidth=1.2,
            linestyle="--",
            alpha=0.75,
            zorder=2,
        )
    )
    clipped_x_min, clipped_x_max, clipped_y_min, clipped_y_max = (
        voronoi_map.clipped_bounds_mm
    )
    if clipped_x_min < clipped_x_max and clipped_y_min < clipped_y_max:
        ax.add_patch(
            Rectangle(
                (clipped_x_min, clipped_y_min),
                clipped_x_max - clipped_x_min,
                clipped_y_max - clipped_y_min,
                fill=False,
                edgecolor="#e89623",
                linewidth=1.6,
                alpha=0.95,
                zorder=2,
            )
        )

    return {
        "field_color": field_color,
        "xlim": (
            field_x_min - margin - goal_depth,
            field_x_max + margin + goal_depth,
        ),
        "ylim": (field_y_min - margin, field_y_max + margin),
    }


def _draw_ssl_vision_geometry(ax, field_geometry, line_color: str) -> None:
    from matplotlib.patches import Arc

    for line in getattr(field_geometry, "field_lines", ()):
        if line is None:
            continue
        thickness = max(1.0, float(getattr(line, "thickness", 10.0)) / 10.0)
        ax.plot(
            [line.p1.x, line.p2.x],
            [line.p1.y, line.p2.y],
            color=line_color,
            linewidth=thickness,
            zorder=1,
        )

    for arc in getattr(field_geometry, "field_arcs", ()):
        if arc is None:
            continue
        thickness = max(1.0, float(getattr(arc, "thickness", 10.0)) / 10.0)
        radius = float(arc.radius)
        ax.add_patch(
            Arc(
                (arc.center.x, arc.center.y),
                2.0 * radius,
                2.0 * radius,
                angle=0.0,
                theta1=degrees(float(arc.a1)),
                theta2=degrees(float(arc.a2)),
                edgecolor=line_color,
                linewidth=thickness,
                zorder=1,
            )
        )


def _field_value(field_geometry, name: str, default: float) -> float:
    value = getattr(field_geometry, name, None)
    if value is None or float(value) <= 0:
        return float(default)
    return float(value)
