import json

from research_sdk.machine_info import machine_info


def test_machine_info_is_json_and_has_core_fields():
    info = machine_info()
    json.dumps(info)
    for key in ("os", "cpu_model", "cpu_logical_cores", "python", "packages"):
        assert key in info
    assert "hostname" not in info
