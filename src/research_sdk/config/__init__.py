"""Static field geometry and planner settings loaded from package YAML files."""

from __future__ import annotations

from importlib.resources import files
from ipaddress import ip_address
from typing import Any

import yaml


_FIELD_KEYS = {
    "team_is_positive",
    "field_length_mm",
    "field_width_mm",
    "defence_x_mm",
    "defence_y_mm",
    "goal_width_mm",
    "goal_depth_mm",
    "robot_radius_mm",
    "safe_margin_mm",
    "ball_radius_mm",
}

_PLANNER_KEYS = {
    "voronoi_boundary_inset_mm",
    "voronoi_density_percent",
    "voronoi_max_density_nodes",
    "voronoi_render_density_percent",
    "voronoi_render_max_density_nodes",
    "voronoi_obstacle_cost_weight",
    "voronoi_horizon_ms",
    "voronoi_target_dead_zone_mm",
    "voronoi_connection_count",
    "voronoi_connection_radius_mm",
    "voronoi_endpoint_reach_mm",
    "voronoi_escape_margin_mm",
    "voronoi_min_escape_step_mm",
}

_NETWORK_KEYS = {
    "multicast_interface_ip",
    "ssl_vision_multicast_group",
    "ssl_vision_port",
    "ssl_vision_tracker_multicast_group",
    "ssl_vision_tracker_port",
    "ssl_game_controller_multicast_group",
    "ssl_game_controller_port",
    "grsim_vision_port",
    "grsim_command_ip",
    "grsim_command_port",
    "robot_telemetry_bind_ip",
    "robot_telemetry_port",
}

_ROBOT_SPEED_KEYS = {
    "max_linear_speed_mps",
    "max_angular_speed_rad_s",
    "max_angular_linear_ratio",
    "flat_kick_speed_mps",
    "chip_kick_speed_mps",
}


def _load_yaml(name: str) -> dict[str, Any]:
    resource = files(__package__).joinpath(name)
    with resource.open("r", encoding="utf-8") as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict):
        raise ValueError(f"{name} must contain a YAML mapping")
    return values


def _validate_keys(name: str, values: dict[str, Any], expected: set[str]) -> None:
    missing = expected - values.keys()
    unknown = values.keys() - expected
    if missing or unknown:
        details = []
        if missing:
            details.append(f"missing: {', '.join(sorted(missing))}")
        if unknown:
            details.append(f"unknown: {', '.join(sorted(unknown))}")
        raise ValueError(f"Invalid {name} ({'; '.join(details)})")


def _positive_number(values: dict[str, Any], key: str) -> float:
    value = values[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"{key} must be a positive number")
    return float(value)


SSL_FIELD_CONFIG = _load_yaml("ssl_field_config.yaml")
PLANNER_VARIABLES = _load_yaml("planner_variables.yaml")
NETWORK_INPUT_CONFIG = _load_yaml("network_input.yaml")
ROBOT_SPEED_CONFIG = _load_yaml("robot_speed_config.yaml")
_validate_keys("ssl_field_config.yaml", SSL_FIELD_CONFIG, _FIELD_KEYS)
_validate_keys("planner_variables.yaml", PLANNER_VARIABLES, _PLANNER_KEYS)
_validate_keys("network_input.yaml", NETWORK_INPUT_CONFIG, _NETWORK_KEYS)
_validate_keys("robot_speed_config.yaml", ROBOT_SPEED_CONFIG, _ROBOT_SPEED_KEYS)

for _dimension_key in _FIELD_KEYS - {"team_is_positive", "safe_margin_mm"}:
    _positive_number(SSL_FIELD_CONFIG, _dimension_key)
if not isinstance(SSL_FIELD_CONFIG["team_is_positive"], bool):
    raise ValueError("team_is_positive must be a boolean")
if not isinstance(SSL_FIELD_CONFIG["safe_margin_mm"], (int, float)):
    raise ValueError("safe_margin_mm must be numeric")
for _planner_key in _PLANNER_KEYS - {"voronoi_obstacle_cost_weight"}:
    _positive_number(PLANNER_VARIABLES, _planner_key)
if not isinstance(PLANNER_VARIABLES["voronoi_obstacle_cost_weight"], (int, float)):
    raise ValueError("voronoi_obstacle_cost_weight must be numeric")
if PLANNER_VARIABLES["voronoi_obstacle_cost_weight"] < 0:
    raise ValueError("voronoi_obstacle_cost_weight must be non-negative")
for _density_key in ("voronoi_density_percent", "voronoi_render_density_percent"):
    if PLANNER_VARIABLES[_density_key] > 100:
        raise ValueError(f"{_density_key} must not exceed 100")
for _integer_key in (
    "voronoi_max_density_nodes",
    "voronoi_render_max_density_nodes",
    "voronoi_horizon_ms",
    "voronoi_connection_count",
):
    if not isinstance(PLANNER_VARIABLES[_integer_key], int):
        raise ValueError(f"{_integer_key} must be an integer")

for _port_key in (key for key in _NETWORK_KEYS if key.endswith("_port")):
    _port = NETWORK_INPUT_CONFIG[_port_key]
    if isinstance(_port, bool) or not isinstance(_port, int) or not 1 <= _port <= 65535:
        raise ValueError(f"{_port_key} must be an integer from 1 to 65535")
for _ip_key in (key for key in _NETWORK_KEYS if key.endswith("_ip")):
    try:
        ip_address(NETWORK_INPUT_CONFIG[_ip_key])
    except ValueError as exc:
        raise ValueError(f"{_ip_key} must be a valid IP address") from exc
for _group_key in (key for key in _NETWORK_KEYS if key.endswith("_group")):
    try:
        _group = ip_address(NETWORK_INPUT_CONFIG[_group_key])
    except ValueError as exc:
        raise ValueError(f"{_group_key} must be a valid multicast IP address") from exc
    if not _group.is_multicast:
        raise ValueError(f"{_group_key} must be a multicast IP address")
_positive_number(ROBOT_SPEED_CONFIG, "max_linear_speed_mps")
_positive_number(ROBOT_SPEED_CONFIG, "max_angular_speed_rad_s")
_angular_ratio = ROBOT_SPEED_CONFIG["max_angular_linear_ratio"]
if (
    isinstance(_angular_ratio, bool)
    or not isinstance(_angular_ratio, (int, float))
    or not 0 <= _angular_ratio <= 1
):
    raise ValueError("max_angular_linear_ratio must be between 0 and 1")
for _speed_key in ("flat_kick_speed_mps", "chip_kick_speed_mps"):
    _speed = ROBOT_SPEED_CONFIG[_speed_key]
    if isinstance(_speed, bool) or not isinstance(_speed, (int, float)) or _speed < 0:
        raise ValueError(f"{_speed_key} must be a non-negative number")

# Field and object geometry.
TEAM_IS_POSITIVE = bool(SSL_FIELD_CONFIG["team_is_positive"])
FIELD_LENGTH_MM = float(SSL_FIELD_CONFIG["field_length_mm"])
FIELD_WIDTH_MM = float(SSL_FIELD_CONFIG["field_width_mm"])
DEFENCE_X_MM = float(SSL_FIELD_CONFIG["defence_x_mm"])
DEFENCE_Y_MM = float(SSL_FIELD_CONFIG["defence_y_mm"])
GOAL_WIDTH_MM = float(SSL_FIELD_CONFIG["goal_width_mm"])
GOAL_HALF_WIDTH_MM = GOAL_WIDTH_MM / 2.0
GOAL_DEPTH_MM = float(SSL_FIELD_CONFIG["goal_depth_mm"])
ROBOT_RADIUS_MM = float(SSL_FIELD_CONFIG["robot_radius_mm"])
SAFE_MARGIN = float(SSL_FIELD_CONFIG["safe_margin_mm"])
BALL_R = float(SSL_FIELD_CONFIG["ball_radius_mm"])

FIELD_X_MIN = -FIELD_LENGTH_MM / 2
FIELD_X_MAX = FIELD_LENGTH_MM / 2
FIELD_Y_MIN = -FIELD_WIDTH_MM / 2
FIELD_Y_MAX = FIELD_WIDTH_MM / 2

# Keep the established uppercase Python API while making YAML the source of truth.
for _key, _value in PLANNER_VARIABLES.items():
    globals()[_key.upper()] = _value

# Clearance is a physical constraint, not an independently tuned planner value.
VORONOI_MIN_CLEARANCE_MM = ROBOT_RADIUS_MM + SAFE_MARGIN

# Network endpoints.
MULTICAST_INTERFACE_IP = str(NETWORK_INPUT_CONFIG["multicast_interface_ip"])
SSL_VISION_MULTICAST_GROUP = str(NETWORK_INPUT_CONFIG["ssl_vision_multicast_group"])
SSL_VISION_PORT = int(NETWORK_INPUT_CONFIG["ssl_vision_port"])
SSL_VISION_TRACKER_MULTICAST_GROUP = str(
    NETWORK_INPUT_CONFIG["ssl_vision_tracker_multicast_group"]
)
SSL_VISION_TRACKER_PORT = int(NETWORK_INPUT_CONFIG["ssl_vision_tracker_port"])
SSL_GAME_CONTROLLER_MULTICAST_GROUP = str(
    NETWORK_INPUT_CONFIG["ssl_game_controller_multicast_group"]
)
SSL_GAME_CONTROLLER_PORT = int(NETWORK_INPUT_CONFIG["ssl_game_controller_port"])
GRSIM_VISION_PORT = int(NETWORK_INPUT_CONFIG["grsim_vision_port"])
GRSIM_COMMAND_IP = str(NETWORK_INPUT_CONFIG["grsim_command_ip"])
GRSIM_COMMAND_PORT = int(NETWORK_INPUT_CONFIG["grsim_command_port"])
ROBOT_TELEMETRY_BIND_IP = str(NETWORK_INPUT_CONFIG["robot_telemetry_bind_ip"])
ROBOT_TELEMETRY_PORT = int(NETWORK_INPUT_CONFIG["robot_telemetry_port"])

# Robot command limits.
ROBOT_MAX_LINEAR_SPEED_MPS = float(ROBOT_SPEED_CONFIG["max_linear_speed_mps"])
ROBOT_MAX_ANGULAR_SPEED_RAD_S = float(ROBOT_SPEED_CONFIG["max_angular_speed_rad_s"])
ROBOT_MAX_ANGULAR_LINEAR_RATIO = float(
    ROBOT_SPEED_CONFIG["max_angular_linear_ratio"]
)
ROBOT_FLAT_KICK_SPEED_MPS = float(ROBOT_SPEED_CONFIG["flat_kick_speed_mps"])
ROBOT_CHIP_KICK_SPEED_MPS = float(ROBOT_SPEED_CONFIG["chip_kick_speed_mps"])

del (
    _density_key,
    _dimension_key,
    _angular_ratio,
    _group,
    _group_key,
    _integer_key,
    _ip_key,
    _key,
    _planner_key,
    _port,
    _port_key,
    _speed,
    _speed_key,
    _value,
)
