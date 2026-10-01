"""Generate the frozen ACRA 2026 six-robot/six-moving-obstacle bank."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from research_sdk.scenario_generator import GeneratorConfig, random_scenario


def generate(output_dir: Path, count: int = 200, seed: int = 0, force: bool = False) -> None:
    """Write a reproducible bank without running the headless simulator."""
    if count < 1:
        raise ValueError("count must be positive")
    existing = sorted(output_dir.glob("*.json")) if output_dir.exists() else []
    if existing and not force:
        raise FileExistsError(
            f"{output_dir} already contains {len(existing)} JSON files; use --force to rework it"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    config = GeneratorConfig(
        robots=6,
        obstacles=6,
        moving_fraction=1.0,
        patrol_fraction=1.0,
        patrol_points=3,
    )
    for path in output_dir.glob("*.json"):
        path.unlink()
    for index in range(count):
        scenario = random_scenario(index, seed=seed, config=config)
        (output_dir / f"{scenario.name}.json").write_text(
            json.dumps(scenario.to_dict(), indent=2), encoding="utf-8"
        )
    print(
        f"generated {count} scenarios in {output_dir} "
        f"(robots=6, obstacles=6, moving_obstacles=6, seed={seed})"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("scenarios/acra2026-200"))
    parser.add_argument("--count", type=int, default=200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    generate(args.output_dir, count=args.count, seed=args.seed, force=args.force)


if __name__ == "__main__":
    main()
