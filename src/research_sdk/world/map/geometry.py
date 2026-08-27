from math import hypot,  atan2, cos, sin

def distance_2_segment(
        point: tuple[float, float],
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> float:
    """Return shortest distance from point to line segment start-end."""
    px, py = point
    sx, sy = start
    ex, ey = end

    dx = ex - sx
    dy = ey - sy

    # Start and end are the same point
    if dx == 0 and dy == 0:
        return hypot(px - sx, py - sy)

    # Project point onto line segment
    t = ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)

    # Clamp projection to segment
    t = max(0.0, min(1.0, t))

    closest_x = sx + t * dx
    closest_y = sy + t * dy

    return hypot(px - closest_x, py - closest_y)

def linear_diff(pos_1:float,pos_2:float) -> float:
    """Return the shortest dist from linear point 1 to linear point 2."""
    return pos_2 - pos_1

def linear_velocity(pos_1: float, pos_2: float, dt_seconds: float) -> float:
    if dt_seconds <= 0:
        raise ValueError("dt_seconds must be positive")
    return(linear_diff(pos_1,pos_2)/dt_seconds)

def angle_diff(angle_1: float, angle_2: float) -> float:
    """Return the shortest signed rotation from object 1 to object 2."""
    diff = angle_2 - angle_1
    return atan2(sin(diff), cos(diff))


def angular_velocity(angle_1: float, angle_2: float, dt_seconds: float) -> float:
    """Return angular velocity in radians per second."""
    if dt_seconds <= 0:
        raise ValueError("dt_seconds must be positive")

    return angle_diff(angle_1, angle_2) / dt_seconds