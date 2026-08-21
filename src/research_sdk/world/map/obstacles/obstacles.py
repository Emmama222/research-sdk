import time
from dataclasses import dataclass
from math import hypot
from research_sdk.world.map.geometry import distance_2_segment, linear_velocity
from research_sdk.config import BALL_R, ROBOT_RADIUS_MM, SAFE_MARGIN

## Standard Robot Radius
R = ROBOT_RADIUS_MM  # mm
MARGIN = SAFE_MARGIN  #mm

@dataclass
class Obstacle:
    timestamp: float  # seconds
    robot_id: int
    isYellow: bool
    pos_mm: tuple[float, float, float]  # mm
    received_at_s: float | None = None  # local Unix time when the frame arrived
    @property
    def radius(self) -> float:
        """The physical radius of robot"""
        return R # hard radius
    @property
    def safe_radius(self) -> float:
        """Returns The adjusted buffer for robot to travel"""
        return self.radius + MARGIN

    def dynamic_1d(self, horizon_ms: int | float) -> float:
        """Returns the dynamic radius of the robot after a given horizon in milliseconds."""
        if horizon_ms < 0:
            raise ValueError("horizon_ms must be non-negative")
        dt_s = horizon_ms/1000
        
        return self.safe_radius * 2  + self.speed_mmps * dt_s
    
    
    def dynamic_radius_0(self, horizon_ms: int | float) -> float:
        """Returns the dynamic radius of the robot after a given horizon in milliseconds."""
        if horizon_ms < 0:
            raise ValueError("horizon_ms must be non-negative")
        dt_s = horizon_ms/1000
        return self.safe_radius  + self.speed_mmps * dt_s

    def age_s(self, now_s: float | None = None) -> float:
        """Return local time elapsed since receipt of this observation."""
        if now_s is None:
            now_s = time.time()
        reference_s = (
            self.received_at_s
            if self.received_at_s is not None
            else self.timestamp
        )
        return max(0.0, now_s - reference_s)

    @property
    def vel_mmps(self):
        return getattr(self,"_vel_mmps",(0.0,0.0))


    def update_vel_from(self, old_obs: "Obstacle") -> tuple[float, float]:
        """Update this observation's velocity from an older observation."""
        if (
            not isinstance(old_obs, Obstacle)
            or old_obs.robot_id != self.robot_id
            or old_obs.isYellow != self.isYellow
            or old_obs.timestamp >= self.timestamp
        ):
            raise ValueError("Expected an older observation of the same robot")

        dt_s = self.timestamp - old_obs.timestamp
        vx = linear_velocity(old_obs.pos_mm[0], self.pos_mm[0], dt_s)
        vy = linear_velocity(old_obs.pos_mm[1], self.pos_mm[1], dt_s)
        self._vel_mmps = (vx, vy)
        return self.vel_mmps

    @property
    def speed_mmps(self) -> float:
        """ speed scalar value in mm/s """
        try :
            return hypot(self.vel_mmps[0], self.vel_mmps[1])
        except (AttributeError, IndexError, TypeError):
            return 0.0


    @property
    def speed_mps(self) -> float:
        """ speed scalar value in m/s """
        return self.speed_mmps/1000

    def predicted_pos(self, horizon_ms: int | float) -> tuple[float, float]:
        """
        Return the predicted world position after a horizon in milliseconds.
        """
        if horizon_ms < 0:
            raise ValueError("horizon_ms must be non-negative")

        dt_s = horizon_ms / 1000.0
        return (
            self.pos_mm[0] + self.vel_mmps[0] * dt_s,
            self.pos_mm[1] + self.vel_mmps[1] * dt_s,
        )


    def vector_to(self, target_pos: tuple[float, float]) -> tuple[float, float]:
        """ returns a position vector [dx,dy] (in mm) from Obstacle to target"""
        dx = target_pos[0] - self.pos_mm[0]
        dy = target_pos[1] - self.pos_mm[1]
        return (dx, dy)

    def dist_to(self, target_pos: tuple[float, float]) -> float:
        """ returns a scalar distance (in mm) from Obstacle to target
        """
        return hypot(*self.vector_to(target_pos))

    def possesses_ball(self, ball_pos) -> bool:
        """ returns True if the ball is within the robot's radius
        """
        if self.dist_to(ball_pos) < self.radius + MARGIN:
            return True
        return False

    def is_target_in_obs(self,target_pos):
        return True if self.dist_to(target_pos) <= self.safe_radius else False

    def clearance_to_path_mm(
        self,
        start_pos_mm: tuple[float, float],
        end_pos_mm: tuple[float, float],
        moving_robot_radius_mm: float = R,
        horizon_ms: int | float = 0,
    ) -> float:
        """
        Checks moving robot (dynamic) radius to the line between start -> end
        Unit : mm

        Args:
            start_pos_mm (tuple[float, float]): the starting position in mm
            end_pos_mm (tuple[float, float]): Ending position (maybe waypoint) in mm
            moving_robot_radius_mm (float, optional): the moving robot's dynamic radius. Defaults to R.

        Returns:
            float: distance to path - moving robot's radius - self.safe_radius
        """
        dist_to_path = distance_2_segment(
            point=self.predicted_pos(horizon_ms),
            start=start_pos_mm,
            end=end_pos_mm,
        )

        return dist_to_path - moving_robot_radius_mm - self.dynamic_radius_0(horizon_ms)


    def intersects_path(
        self,
        start_pos_mm: tuple[float, float],
        end_pos_mm: tuple[float, float],
        extra_clearance_mm: float = 0.0,
        horizon_ms: int | float = 0,
    ) -> bool:

        return self.clearance_to_path_mm(
            start_pos_mm=start_pos_mm,
            end_pos_mm=end_pos_mm,
            horizon_ms=horizon_ms,
        ) <= extra_clearance_mm

# Todo : add a  test to use snapshot -> generate obstacle object
if __name__ == "__main__":
    o1 = Obstacle(time.monotonic(),0, True, (0.0, 0.0, 0.0))
    time.sleep(1)
    o2 = Obstacle(time.monotonic(),0, True, (1.0, 0.0, 0.0))
    o2.update_vel_from(o1)
    print(o2.vel_mmps,o2.predicted_pos(1000))

    print(o1.is_target_in_obs([110.0,0.0]))

    print(o1.is_target_in_obs([140.0,0.0]))
