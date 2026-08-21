# Robot Commands
import math
import time

from research_sdk.robot.constants import MAX_SPEED


def _clamp_velocity(vx: float, vy: float, max_speed: float = MAX_SPEED) -> tuple[float, float]:
    """Scale (vx, vy) so magnitude does not exceed max_speed."""
    speed = math.hypot(vx, vy)
    if speed <= 0 or speed <= max_speed:
        return float(vx), float(vy)
    scale = max_speed / speed
    return vx * scale, vy * scale


class RobotCommand():
    __slots__ = ('time_set', 'isYellow', 'robot_id', 'vx', 'vy', 'w', 'kick', 'dribble', 'time_origin')
    def __init__(self, robot_id : int, vx : float=0.0, vy: float=0.0, w : float=0.0, kick : int=0, dribble : int=0, time_origin : float= 0.0, isYellow: bool = True):
        """Robot Command (Previously know as Command)
            Object for initialise commands, encode / decode strings for UDP transportation.
        Args:
            robot_id (int) : wanted Robot ID
            vx (float): wanted velocity for x direction
            vy (float): wanted velocity for y direction
            w (float): wanted angular velocity (radians)
            kick (int): wanted to kick : (0/1)
            dribble (int): wanted to dribble : (0,1)
            time_origin (float): when was this packet first created. Default = 0.0
            
        Params:
            time_set(time.time): time of packet generated
        """
        self.time_set: float = time.time()
        self.isYellow: bool = isYellow
        self.robot_id: int = int(robot_id)
        self.vx, self.vy = _clamp_velocity(float(vx), float(vy))
        self.w: float = float(w)
        self.kick: int = int(kick)
        self.dribble: int = int(dribble)
        self.time_origin: float = float(time_origin)
    
    def to_dict(self):
        return {
            "robot_id": self.robot_id,
            "vx" : self.vx,
            "vy" : self.vy,
            "w" : self.w,
            "kick" : self.kick,
            "dribble" : self.dribble,
            "isYellow" : self.isYellow
        }
    
    def __repr__(self):
        """repr 
            This is a magic function
            It is the representation of Command Class (use for debuging)
        
        return: 
          string : In debuging format of RobotCommand Class objct
        """
        return f"{self.time_set=},{self.time_origin=}| {self.robot_id=} | {self.vx=} , {self.vy=}, {self.w=} | {self.kick=} {self.dribble=}"
            
    def __str__(self) -> str:
        # the string will not include isYellow
        return f"{self.robot_id} {self.vx} {self.vy} {self.w} {self.kick} {self.dribble} {self.time_set}"
        
    def encode(self) -> bytes:
        """encode
            Encodes Command object into bytes
            
        Returns:
            bytes: byte data for sending
        
        """
        return str(self).encode('utf-8')
    
    @classmethod
    def decode(cls,command_msg:str|bytes) -> object:
        """decode
            decode and stores the Command to an object *This needs to be a class method
        Args:
            command_msg (str|bytes): message received upon UDP (in the form of string or bytes)
            
        Params: 
            args (arguments): list of arguments to be parsed into creating an RobotCommand Object

        Returns:
            object: RobotCommand object for robot to access
        """
        ## if bytes, decode into string first
        if isinstance(command_msg, bytes):
            command_msg = command_msg.decode()

        robot_id, vx, vy, w, kick, dribble, time_origin = command_msg.split(" ")

        return RobotCommand(int(robot_id), float(vx), float(vy), float(w), int(kick), int(dribble), float(time_origin))
    
  