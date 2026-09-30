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


def xy(p: list[float] | tuple[float, float]) -> tuple[int, int]:
    return (int(PAD + (p[0] + FIELD_L / 2) * (W - 2 * PAD) / FIELD_L),
            int(PAD + (FIELD_W / 2 - p[1]) * (H - 2 * PAD) / FIELD_W))


def radius(r: float) -> int:
    return max(2, int(r * (W - 2 * PAD) / FIELD_L))


def render(path: Path, out: Path) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    image = Image.new("RGB", (W, H), "white")
    draw = ImageDraw.Draw(image)
    draw.rectangle((PAD, PAD, W - PAD, H - PAD), outline="#c7ccd4", width=2)
    mid = xy((0, 0))
    draw.line((mid[0], PAD, mid[0], H - PAD), fill="#e3e6ea", width=1)

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
                  f"Y{obstacle.get('obstacle_id', '?')} patrol",
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

    draw.text((PAD + 8, H - PAD + 4),
              "Blue B#: start → goal route   |   Yellow Y#: moving patrol   |   "
              "filled circle = start, open square = end goal",
              fill="#3f4650")

    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out, format="PNG")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario-dir", type=Path, default=Path("scenarios/acra2026-200"))
    parser.add_argument("--scenario-output", type=Path, default=Path("scenario_png"))
    args = parser.parse_args()
    files = sorted(args.scenario_dir.glob("*.json"))
    for path in files:
        render(path, args.scenario_output / f"{path.stem}.png")
    print(f"rendered {len(files)} scenario PNGs to {args.scenario_output}")


if __name__ == "__main__":
    main()
