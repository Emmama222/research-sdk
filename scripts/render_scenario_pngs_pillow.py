"""Dependency-light scenario-bank PNG renderer.

This is a fallback for environments where the repository venv cannot be
started. The canonical renderer remains ``scripts/report_batch.py``.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


FIELD_L, FIELD_W = 9000.0, 6000.0
ROBOT_R = 90.0
W, H = 1080, 760
PAD = 34
HEADER = 46  # strip above the field with the scenario name and counts


def xy(p: list[float] | tuple[float, float]) -> tuple[int, int]:
    return (int(PAD + (p[0] + FIELD_L / 2) * (W - 2 * PAD) / FIELD_L),
            int(HEADER + PAD + (FIELD_W / 2 - p[1]) * (H - 2 * PAD) / FIELD_W))


def _font(size: int, bold: bool = False):
    """DejaVu Sans from the system or from matplotlib's bundled copy; Pillow default otherwise."""
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        pass
    try:
        from matplotlib import font_manager

        path = font_manager.findfont(font_manager.FontProperties(
            family="DejaVu Sans", weight="bold" if bold else "normal"))
        return ImageFont.truetype(path, size)
    except Exception:  # noqa: BLE001 - any failure falls back to the default bitmap font
        return ImageFont.load_default()


def is_moving(obstacle: dict) -> bool:
    """A patrol route or a non-zero velocity makes an obstacle moving; otherwise static."""
    velocity = obstacle.get("velocity_mmps") or [0.0, 0.0]
    return bool(obstacle.get("patrol_waypoints")) or any(abs(float(v)) > 1e-9 for v in velocity)


def summary_line(data: dict, name: str) -> tuple[str, str]:
    obstacles = data.get("obstacles", [])
    moving = [o for o in obstacles if is_moving(o)]
    speeds = [float(o["patrol_speed_mmps"]) for o in moving if o.get("patrol_speed_mmps")]
    speed = f"   |   patrol speed {min(speeds):.0f}–{max(speeds):.0f} mm/s" if speeds else ""
    counts = (f"Robots (start → goal): {len(data.get('robots', []))}   |   "
              f"Moving obstacles: {len(moving)}   |   Static obstacles: {len(obstacles) - len(moving)}"
              f"{speed}")
    return name, counts


def radius(r: float) -> int:
    return max(2, int(r * (W - 2 * PAD) / FIELD_L))


def render(path: Path, out: Path) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    image = Image.new("RGB", (W, H + HEADER), "white")
    draw = ImageDraw.Draw(image)
    title, counts = summary_line(data, data.get("name") or path.stem)
    title_font, body_font, legend_font = _font(17, bold=True), _font(13), _font(11)
    draw.text((PAD, 8), title, fill="#0b0b0b", font=title_font)
    draw.text((PAD, 30), counts, fill="#52514e", font=body_font)
    top, bottom = HEADER + PAD, HEADER + H - PAD
    draw.rectangle((PAD, top, W - PAD, bottom), outline="#c7ccd4", width=2)
    mid = xy((0, 0))
    draw.line((mid[0], top, mid[0], bottom), fill="#e3e6ea", width=1)

    # Yellow entities are the moving/patrolling obstacles; blue entities are
    # the robots whose start-to-goal routes are being evaluated.
    for obstacle in data.get("obstacles", []):
        pos = obstacle["position_mm"]
        centre = xy(pos)
        r = float(obstacle.get("radius_mm", ROBOT_R))
        rr = radius(r)
        inflated = radius(r + ROBOT_R)
        waypoints = obstacle.get("patrol_waypoints") or []
        route = [pos] + waypoints
        if len(route) > 1:
            draw.line([xy(p) for p in route], fill="#d6a900", width=2)
        draw.ellipse((centre[0] - inflated, centre[1] - inflated,
                      centre[0] + inflated, centre[1] + inflated),
                     outline="#d6a900", width=1)
        draw.ellipse((centre[0] - rr, centre[1] - rr,
                      centre[0] + rr, centre[1] + rr), fill="#f2c900",
                     outline="#9c7800", width=1)
        draw.text((centre[0] + rr + 4, centre[1] - 9),
                  f"Y{obstacle.get('obstacle_id', '?')} {'patrol' if is_moving(obstacle) else 'static'}",
                  fill="#806200")

    for robot in data.get("robots", []):
        start, target = robot["start_mm"], robot["target_mm"]
        s, t = xy(start), xy(target)
        draw.line((s[0], s[1], t[0], t[1]), fill="#256abf", width=3)
        rr = radius(ROBOT_R)
        draw.ellipse((s[0] - rr, s[1] - rr, s[0] + rr, s[1] + rr),
                     fill="#3987e5", outline="#184f95", width=1)
        draw.text((s[0] + rr + 4, s[1] - 9),
                  f"B{robot.get('robot_id', '?')} start", fill="#184f95")
        # Open square = end goal; the label makes the start/end distinction
        # unambiguous even when routes cross.
        goal_r = 8
        draw.rectangle((t[0] - goal_r, t[1] - goal_r,
                        t[0] + goal_r, t[1] + goal_r),
                       outline="#184f95", width=3)
        draw.text((t[0] + goal_r + 4, t[1] - 9),
                  f"B{robot.get('robot_id', '?')} goal", fill="#184f95")

    draw.text((PAD + 8, HEADER + H - PAD + 4),
              "Blue B#: robot, start → goal route   |   Yellow Y#: obstacle (patrol or static)   |   "
              "filled circle = start, open square = end goal",
              fill="#3f4650", font=legend_font)

    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out, format="PNG")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario-dir", type=Path, default=Path("scenarios/acra2026-200"))
    parser.add_argument("--scenario-output", type=Path, default=Path("scenario_png/acra2026-200"))
    args = parser.parse_args()
    files = sorted(args.scenario_dir.glob("*.json"))
    for path in files:
        render(path, args.scenario_output / f"{path.stem}.png")
    print(f"rendered {len(files)} scenario PNGs to {args.scenario_output}")


if __name__ == "__main__":
    main()
