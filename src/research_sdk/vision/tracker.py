import numpy as np
from enum import Enum


class Capability (Enum):
    CAPABILITY_UNKNOWN = 0
    CAPABILITY_DETECT_FLYING_BALLS = 1
    CAPABILITY_DETECT_MULTIPLE_BALLS = 2
    CAPABILITY_DETECT_KICKED_BALLS = 3


class Vector2:
    __slots__ = ('x', 'y')

    def __init__(self,x,y):
        self.x = float(x)
        self.y = float(y)

    @property
    def array (self)-> np.ndarray[float]:
        return np.array([self.x, self.y], dtype=float)

class Vector3:
    __slots__ = ('x', 'y', 'z')

    def __init__(self,x,y,z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @property
    def array (self)-> np.ndarray[float]:
        return np.array([self.x, self.y, self.z], dtype=float)

class TrackedBall():
    __slots__ = ('pos', 'vel', 'visiblity')

    def __init__(self, pos:Vector3, vel:Vector3=None,visibility:float=None):
        self.pos:Vector3= pos
        self.vel:Vector3= vel
        self.visiblity:float= visibility

class KickedBall():
    __slots__ = ('pos', 'vel', 'start_timestamp', 'stop_timestamp', 'stop_pos', 'robot_id')

    def __init__(self ,pos:Vector2, vel:Vector3, start_timestamp:float,stop_timestamp:float=None,
                 stop_pos:Vector2=None,robot_id:int=None):
        self.pos:Vector2 = pos
        ## The initial velocity [m/s] with which the ball was kicked
        self.vel:Vector3 = vel
        ## The unix timestamp [s] when the kick was performed
        self.start_timestamp:float= start_timestamp
        ## The predicted unix timestamp [s] when the ball comes to a stop
        self.stop_timestamp:float = stop_timestamp
        ## The predicted position [m] at which the ball will come to a stop
        self.stop_pos:Vector2 = stop_pos
        ## The robot that kicked the ball
        self.robot_id : int = robot_id

class TrackedRobot():
    __slots__ = ('robot_id', 'pos', 'orientation', 'vel', 'vel_angular', 'visibility')

    def __init__(self,robot_id:int,pos:Vector2,orientation:float,vel:Vector2,vel_angular:float,visibility:float ):
        self.robot_id:int = robot_id
        ## The position [m] in the ssl-vision coordinate system
        self.pos:Vector2 = pos
        ## The orientation [rad] in the ssl-vision coordinate system
        self.orientation:float = orientation
        ## The velocity [m/s] in the ssl-vision coordinate system
        self.vel:Vector2 = vel
        ## The angular velocity [rad/s] in the ssl-vision coordinate system

        self.vel_angular:float = vel_angular
        ## The visibility of the robot
        ## A value between 0 (not visible) and 1 (visible)
        ## The exact implementation depends on the source software
        self.visibility:float = visibility

class TrackedFrame():
    __slots__ = ('frame_number', 'timestamp', 'balls', 'robots', 'kicked_ball', 'capability')

    def __init__(self,frame_number:int,timestamp:float,balls:list[TrackedBall],
                 robots:list[TrackedRobot],kicked_ball:KickedBall=None,capability=None
                 ):
        ## A monotonous increasing frame counter
        self.frame_number:int = frame_number
        ## The unix timestamp in [s] of the data
        ## If timestamp is larger than timestamp_captured, the source has applied a prediction already
        self.timestamp:float = timestamp

        ## The list of detected balls
        ## The first ball is the primary one
        ## Sources may add additional balls based on their capabilities
        self.balls:list[TrackedBall] = balls
        ## The list of detected robots of both teams
        self.robots:list[TrackedRobot] = robots
        ## Information about a kicked ball, if the ball was kicked by a robot and is still moving
        ## Note: This field is optional. Some source implementations might not set this at any time
        self.kicked_ball:KickedBall = kicked_ball

        ## List of capabilities of the source implementation
        self.capability:list[Capability] = capability if capability is not None else []


class TrackerWrapperPacket() :
    def __init__(self,uuid:str,source_name:str=None,tracked_frame:TrackedFrame=None):

        ## A random UUID of the source that is kept constant at the source while running
        ## If multiple sources are broadcasting to the same network, this id can be used to identify individual sources
        self.uuid:str = uuid
        ## The name of the source software that is producing this messages.
        self.source_name = source_name
        ## The tracked frame
        self.tracked_frame = tracked_frame
