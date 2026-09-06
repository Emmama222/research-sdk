from math import pi

import pytest

from research_sdk.world.shapes import OrientedRectangle


def test_motion_rectangle_uses_milliseconds_and_is_centred_on_displacement() -> None:
    rectangle = OrientedRectangle.from_motion(
        (100.0, 200.0),
        (1000.0, 0.0),
        horizon_ms=250.0,
        radius_mm=90.0,
        buffer_mm=30.0,
    )

    assert rectangle.center_mm == pytest.approx((225.0, 200.0))
    assert rectangle.length_mm == pytest.approx(490.0)
    assert rectangle.width_mm == pytest.approx(242.5)
    assert rectangle.orientation_rad == pytest.approx(0.0)


def test_motion_rectangle_aligns_with_velocity() -> None:
    rectangle = OrientedRectangle.from_motion(
        (0.0, 0.0),
        (0.0, 500.0),
        horizon_ms=400.0,
        radius_mm=90.0,
        buffer_mm=10.0,
    )

    assert rectangle.center_mm == pytest.approx((0.0, 100.0))
    assert rectangle.orientation_rad == pytest.approx(pi / 2.0)
    assert rectangle.length_mm == pytest.approx(400.0)
    assert rectangle.width_mm == pytest.approx(202.0)


def test_stationary_rectangle_uses_supplied_robot_heading() -> None:
    rectangle = OrientedRectangle.from_motion(
        (30.0, -20.0),
        (0.0, 0.0),
        horizon_ms=500.0,
        radius_mm=90.0,
        buffer_mm=30.0,
        stationary_orientation_rad=0.75,
    )

    assert rectangle.center_mm == (30.0, -20.0)
    assert rectangle.length_mm == pytest.approx(240.0)
    assert rectangle.width_mm == pytest.approx(240.0)
    assert rectangle.orientation_rad == pytest.approx(0.75)


def test_rotated_rectangle_corners_and_point_containment() -> None:
    rectangle = OrientedRectangle((0.0, 0.0), 4.0, 2.0, pi / 2.0)

    expected = ((1.0, -2.0), (1.0, 2.0), (-1.0, 2.0), (-1.0, -2.0))
    for actual_corner, expected_corner in zip(rectangle.corners_mm, expected):
        assert actual_corner == pytest.approx(expected_corner)
    assert rectangle.contains_point((0.9, 1.9))
    assert rectangle.contains_point((1.0, 2.0))
    assert not rectangle.contains_point((1.1, 0.0))


def test_segment_intersection_handles_crossing_touching_and_clear_segments() -> None:
    rectangle = OrientedRectangle((0.0, 0.0), 4.0, 2.0)

    assert rectangle.intersects_segment((-3.0, 0.0), (3.0, 0.0))
    assert rectangle.intersects_segment((-3.0, 1.0), (3.0, 1.0))
    assert not rectangle.intersects_segment((-3.0, 2.0), (3.0, 2.0))
    assert rectangle.clearance_to_segment((-3.0, 2.0), (3.0, 2.0)) == pytest.approx(1.0)


def test_inflation_adds_clearance_to_every_side() -> None:
    original = OrientedRectangle((4.0, 5.0), 100.0, 40.0, 0.3)
    inflated = original.inflated(20.0)

    assert inflated.center_mm == original.center_mm
    assert inflated.orientation_rad == original.orientation_rad
    assert inflated.length_mm == pytest.approx(140.0)
    assert inflated.width_mm == pytest.approx(80.0)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    (
        ({"horizon_ms": -1.0}, "horizon_ms"),
        ({"radius_mm": 0.0}, "radius_mm"),
        ({"buffer_mm": -1.0}, "buffer_mm"),
        ({"lateral_growth_factor": -0.01}, "lateral_growth_factor"),
    ),
)
def test_motion_rectangle_rejects_invalid_dimensions(kwargs, message: str) -> None:
    values = {
        "horizon_ms": 250.0,
        "radius_mm": 90.0,
        "buffer_mm": 30.0,
        "lateral_growth_factor": 0.01,
        **kwargs,
    }

    with pytest.raises(ValueError, match=message):
        OrientedRectangle.from_motion((0.0, 0.0), (100.0, 0.0), **values)
