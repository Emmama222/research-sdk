from inspect import signature

import research_sdk.config as config
from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.ui.renderer import Renderer
from research_sdk.world.map.voronoi.voronoi_generator import generate_bounded_voronoi_map
from research_sdk.world.map.world_map import WorldMap
from research_sdk.world.scene import FieldDimensions
from research_sdk.network.ssl_sockets import GameControl, Vision, VisionTracker, grSimSender
from research_sdk.network.grSimPacketFactory import grSimPacketFactory


def test_ssl_field_yaml_is_the_runtime_source_of_truth() -> None:
    raw = config.SSL_FIELD_CONFIG

    assert config.FIELD_LENGTH_MM == float(raw["field_length_mm"])
    assert config.FIELD_WIDTH_MM == float(raw["field_width_mm"])
    assert config.GOAL_WIDTH_MM == float(raw["goal_width_mm"])
    assert config.GOAL_HALF_WIDTH_MM == config.GOAL_WIDTH_MM / 2.0
    assert config.GOAL_DEPTH_MM == float(raw["goal_depth_mm"])
    assert config.ROBOT_RADIUS_MM == float(raw["robot_radius_mm"])
    assert config.BALL_R == float(raw["ball_radius_mm"])
    assert config.VORONOI_MIN_CLEARANCE_MM == (
        float(raw["robot_radius_mm"]) + float(raw["safe_margin_mm"])
    )

    field = FieldDimensions()
    assert field.field_length == config.FIELD_LENGTH_MM
    assert field.field_width == config.FIELD_WIDTH_MM


def test_planner_yaml_supplies_runtime_defaults() -> None:
    raw = config.PLANNER_VARIABLES
    planner = VoronoiDijkstraPlanner()

    assert planner.horizon_ms == raw["voronoi_horizon_ms"]
    assert planner.density_percent == raw["voronoi_density_percent"]
    assert planner.max_density_nodes == raw["voronoi_max_density_nodes"]
    assert planner.obstacle_cost_weight == raw["voronoi_obstacle_cost_weight"]
    assert planner.boundary_inset_mm == raw["voronoi_boundary_inset_mm"]
    assert WorldMap().horizon_ms == raw["voronoi_horizon_ms"]
    assert Renderer().prediction_horizon_ms == raw["voronoi_horizon_ms"]


def test_low_level_generator_defaults_are_yaml_backed() -> None:
    parameters = signature(generate_bounded_voronoi_map).parameters

    assert parameters["field_length_mm"].default == config.FIELD_LENGTH_MM
    assert parameters["field_width_mm"].default == config.FIELD_WIDTH_MM
    assert parameters["boundary_inset_mm"].default == config.VORONOI_BOUNDARY_INSET_MM
    assert parameters["density_percent"].default == config.VORONOI_RENDER_DENSITY_PERCENT
    assert (
        parameters["max_density_nodes"].default
        == config.VORONOI_RENDER_MAX_DENSITY_NODES
    )
    assert (
        parameters["obstacle_cost_weight"].default
        == config.VORONOI_OBSTACLE_COST_WEIGHT
    )


def test_network_yaml_supplies_ssl_socket_defaults() -> None:
    raw = config.NETWORK_INPUT_CONFIG

    assert signature(Vision).parameters["port"].default == raw["ssl_vision_port"]
    assert (
        signature(Vision).parameters["group"].default
        == raw["ssl_vision_multicast_group"]
    )
    assert (
        signature(VisionTracker).parameters["port"].default
        == raw["ssl_vision_tracker_port"]
    )
    assert (
        signature(GameControl).parameters["port"].default
        == raw["ssl_game_controller_port"]
    )
    assert signature(grSimSender).parameters["ip"].default == raw["grsim_command_ip"]
    assert signature(grSimSender).parameters["port"].default == raw["grsim_command_port"]
    assert config.GRSIM_VISION_PORT == raw["grsim_vision_port"]
    assert config.ROBOT_TELEMETRY_PORT == raw["robot_telemetry_port"]


def test_robot_speed_yaml_controls_commands_and_grsim_packets() -> None:
    raw = config.ROBOT_SPEED_CONFIG
    assert config.ROBOT_MAX_LINEAR_SPEED_MPS == raw["max_linear_speed_mps"]
    assert config.ROBOT_MAX_ANGULAR_SPEED_RAD_S == raw["max_angular_speed_rad_s"]
    assert config.ROBOT_MAX_ANGULAR_LINEAR_RATIO == raw["max_angular_linear_ratio"]
    packet = grSimPacketFactory.robot_command(robot_id=1, kick=True)
    robot_command = packet.commands.robot_commands[0]
    assert robot_command.kickspeedx == raw["flat_kick_speed_mps"]
    assert robot_command.kickspeedz == raw["chip_kick_speed_mps"]
