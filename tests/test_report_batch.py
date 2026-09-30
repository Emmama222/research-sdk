import json

import pytest

from scripts.report_batch import render_scenario_folder


def _scenario(path, name: str) -> None:
    path.write_text(
        json.dumps(
            {
                "name": name,
                "robots": [
                    {
                        "robot_id": 0,
                        "is_yellow": False,
                        "start_mm": [-1000.0, 0.0],
                        "target_mm": [1000.0, 0.0],
                    }
                ],
                "obstacles": [],
            }
        ),
        encoding="utf-8",
    )


def test_render_scenario_folder_exports_every_json_in_sorted_order(tmp_path) -> None:
    source = tmp_path / "scenarios"
    output = tmp_path / "scenario_png"
    source.mkdir()
    _scenario(source / "scenario-b.json", "scenario-b")
    _scenario(source / "scenario-a.json", "scenario-a")

    rendered = render_scenario_folder(source, output)

    assert [path.name for path in rendered] == ["scenario-a.png", "scenario-b.png"]
    assert all(path.is_file() and path.stat().st_size > 0 for path in rendered)


def test_render_scenario_folder_supports_an_explicit_limit(tmp_path) -> None:
    source = tmp_path / "scenarios"
    output = tmp_path / "scenario_png"
    source.mkdir()
    _scenario(source / "scenario-a.json", "scenario-a")
    _scenario(source / "scenario-b.json", "scenario-b")

    rendered = render_scenario_folder(source, output, limit=1)

    assert [path.name for path in rendered] == ["scenario-a.png"]


def test_render_scenario_folder_rejects_zero_limit(tmp_path) -> None:
    with pytest.raises(ValueError, match="positive"):
        render_scenario_folder(tmp_path, tmp_path / "scenario_png", limit=0)
