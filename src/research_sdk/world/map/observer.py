class FieldAnalyzer:
    def __init__(self, world_map) -> None:
        self.map = world_map

    def is_path_blocked(
        self,
        start_xy,
        goal_xy,
        clearance_radius=0.0,
        horizon_ms=None,
    ) -> bool:
        return not self.map.is_path_free(
            start_pos=start_xy,
            end_pos=goal_xy,
            clearance=clearance_radius,
            horizon_ms=horizon_ms,
        )

    def score_confidence(self, robot_id, isYellow) -> float:
        """
        Estimate shooting confidence.

        This remains a placeholder until its scoring inputs and weights are
        defined.
        """
        return 1.0

    def pass_confidence(self, robot_id1, robot_id2, isYellow) -> float:
        """
        Estimate passing confidence.

        This remains a placeholder until its scoring inputs and weights are
        defined.
        """
        return 1.0

    def ball_trajectory(self, horizon_ms=20):
        """Return the predicted ball position and velocity."""
        return self.map.get_ball_trajectory(horizon_ms)

    def robot_trajectory(self, robot_id, isYellow, horizon_ms=20):
        """Return the predicted robot position and velocity."""
        return self.map.get_robot_trajectory(
            robot_id,
            isYellow,
            horizon_ms,
        )
