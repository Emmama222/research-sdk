from research_sdk.ui.execution.checkpoints import CheckpointRecord, CheckpointStore


def checkpoint() -> CheckpointRecord:
    return CheckpointRecord.create(
        run_id="run-1",
        checkpoint_id="cp-1",
        parent_run_id=None,
        scenario_name="course",
        scenario_hash="abc",
        triggers=({"is_yellow": True, "robot_id": 1, "reached_waypoint_index": 1},),
        robots=({"is_yellow": True, "robot_id": 1, "pose": [1.0, 2.0, 0.0]},),
        ball=None,
        waypoint_indexes={"Y1": 2},
        velocity_owner="Planner A",
        path_ids={"Y1": "path-1"},
        metrics={"elapsed_ms": 50.0},
        state="RUNNING",
    )


def test_checkpoint_json_round_trip_and_index(tmp_path) -> None:
    store = CheckpointStore(tmp_path / "events.jsonl")
    record = checkpoint()

    store.append(record)

    assert store.list() == (record,)
    assert store.get("cp-1") == record


def test_checkpoint_store_preserves_non_checkpoint_events(tmp_path) -> None:
    path = tmp_path / "events.jsonl"
    path.write_text('{"event":"run_started"}\n', encoding="utf-8")
    store = CheckpointStore(path)
    record = checkpoint()

    store.append(record)

    assert len(path.read_text(encoding="utf-8").splitlines()) == 2
    assert store.list() == (record,)
