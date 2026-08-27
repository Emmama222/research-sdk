from research_sdk.network.proto2 import grSim_Commands_pb2,grSim_Packet_pb2,grSim_Replacement_pb2
from typing import Optional

import time
from research_sdk.config import ROBOT_CHIP_KICK_SPEED_MPS, ROBOT_FLAT_KICK_SPEED_MPS

class grSimPacketFactory():
    ## responsibility of class : generate grSim packet with data provided
    
    # class constants : 
    KICK_SPEED_X = ROBOT_FLAT_KICK_SPEED_MPS
    KICK_SPEED_Z = ROBOT_CHIP_KICK_SPEED_MPS
    
    
    @classmethod
    def robot_command(cls,robot_id,vx=0.0,vy=0.0,w=0.0,kick=False,dribble=False,isYellow=True):
        # Inlined protobuf creation for hot path performance
        robot_cmd = grSim_Commands_pb2.grSim_Robot_Command(
            id=int(robot_id),
            kickspeedx=float(cls.KICK_SPEED_X * kick),
            kickspeedz=float(cls.KICK_SPEED_Z * kick),
            veltangent=float(vx),
            velnormal=float(vy),
            velangular=float(w),
            spinner=bool(dribble),
            wheelsspeed=False
        )
        command = grSim_Commands_pb2.grSim_Commands(
            timestamp=time.time(),
            isteamyellow=bool(isYellow),
            robot_commands=[robot_cmd]
        )
        return grSim_Packet_pb2.grSim_Packet(commands=command)
    
    @classmethod
    def robot_replacement_command(cls,x:float,y:float,orientation:float,
                                  robot_id:int,isYellow:bool
                                  ) -> grSim_Packet_pb2.grSim_Packet:
        # creates a new robot replacement command
        cmd = cls._grSim_RobotReplacement_wrapper(x,y,orientation,robot_id,isYellow)
        # wraps it with the replacement 
        replacement = cls._grSim_Replacement_wrapper(robots=[cmd])
        # wraps it into a packet 
        packet = cls._grSim_packet_wrapper(replacement=replacement)
        # returns the packet
        return packet

    @classmethod
    def scenario_replacement_command(cls, robots) -> grSim_Packet_pb2.grSim_Packet:
        """Build one replacement packet for every robot in a scenario.

        Coordinates must already be converted to grSim metres by the caller.
        """
        replacements = [
            cls._grSim_RobotReplacement_wrapper(
                robot["x"],
                robot["y"],
                robot.get("orientation", 0.0),
                robot["robot_id"],
                robot["isYellow"],
            )
            for robot in robots
        ]
        if not replacements:
            raise ValueError("A scenario replacement requires at least one robot")
        replacement = cls._grSim_Replacement_wrapper(robots=replacements)
        return cls._grSim_packet_wrapper(replacement=replacement)
        
    @classmethod
    def ball_replacement_command(cls,x:float,y:float,
                                 vx:float=0.0,vy:float=0.0
                                 ) -> grSim_Packet_pb2.grSim_Packet:
        # creates new ball replacement command
        cmd = cls._grSim_BallReplacement_wrapper(x,y,vx,vy)
        # wraps it with replacement wrapper 
        replacement = cls._grSim_Replacement_wrapper(ball=cmd, robots=None)
        # wraps into a packet
        packet = cls._grSim_packet_wrapper(replacement=replacement)
        # returns the packet
        return packet
    
    ### handler function ###
    @classmethod
    def _grSimRobotCommand_wrapper(cls,robot_id:int,vx: float,vy: float,w: float,kick:bool,dribble:bool) -> object:
        # converts into grSim_Robot_Command (protobuff object)
        fields = {
            "id" : int(robot_id), 
            "kickspeedx" : float(cls.KICK_SPEED_X*kick),
            "kickspeedz" : float(cls.KICK_SPEED_Z*kick), 
            "veltangent" : float(vx), 
            "velnormal"  : float(vy), 
            "velangular" : float(w), 
            "spinner"    : bool(dribble), 
            "wheelsspeed": False
        }
        
        return grSim_Commands_pb2.grSim_Robot_Command(**fields)
        
    
    @staticmethod
    def _grSimCommand_wrapper(is_yellow:bool,    
        robot_commands: list[grSim_Commands_pb2.grSim_Robot_Command]
        ) -> grSim_Commands_pb2.grSim_Commands:
        ## this requires is_yellow input
        # Converts into grSim_Commands (protobuff object)    
        if robot_commands is None:
            raise ValueError("robot_commands must be provided (non-empty).")
        elif not isinstance(robot_commands,list):
            raise AttributeError("robot_commands needs to be a list")

        if len(robot_commands) == 0:
            raise ValueError("robot_commands must be non-empty.")
                
        return grSim_Commands_pb2.grSim_Commands(
            timestamp=time.time(),
            isteamyellow=bool(is_yellow),
            robot_commands=robot_commands
        )
    
    
    @staticmethod
    def _grSim_RobotReplacement_wrapper(x,y,orientation,robot_id,isYellow
                    ) -> grSim_Replacement_pb2.grSim_RobotReplacement:
        fields = {
            "x"  : float(x),
            "y"  : float(y),
            "dir": float(orientation),
            "id": int(robot_id),
            "yellowteam": bool(isYellow),
            "turnon": True,
        }
        
        return grSim_Replacement_pb2.grSim_RobotReplacement(**fields)

    @staticmethod
    def _grSim_BallReplacement_wrapper( x: Optional[float]  = None,
                                        y: Optional[float]  = None,
                                        vx: Optional[float] = None,
                                        vy: Optional[float] = None,
                            ) -> grSim_Replacement_pb2.grSim_BallReplacement:

        fields = {
            "x" : float(x),
            "y" : float(y),
            "vx": float(vx),
            "vy": float(vy),
        }

        clean = {k: v for k, v in fields.items() if v is not None}

        return grSim_Replacement_pb2.grSim_BallReplacement(**clean)
        
        
    @staticmethod    
    def _grSim_Replacement_wrapper(ball:Optional[grSim_Replacement_pb2.grSim_BallReplacement]=None,
                                   robots:list[grSim_Replacement_pb2.grSim_RobotReplacement]=None):
        if ball is None and robots is None:
            raise ValueError("Both cannot be none")
        
        if robots is None:
            robots = list() # creates an empty list
        elif not isinstance(robots,list) :
            raise TypeError ("need robot_replacement in list")        
            
        fields = {
            "ball": ball,
            "robots": robots
        }
        
        clean = {k: v for k, v in fields.items() if v is not None}
        
        return grSim_Replacement_pb2.grSim_Replacement(**clean)
    
    
    @staticmethod    
    def _grSim_packet_wrapper(commands:grSim_Commands_pb2.grSim_Commands=None,
                              replacement:grSim_Replacement_pb2.grSim_Replacement=None
                              ) -> grSim_Packet_pb2.grSim_Packet:
        
        if commands is None and replacement is None:
            raise ValueError ("Both cannot be None")
        fields = {
            "commands" : commands,
            "replacement" : replacement
        }
        
        clean = {k: v for k, v in fields.items() if v is not None}
        return grSim_Packet_pb2.grSim_Packet(**clean)


if __name__ == "__main__":
    from research_sdk.network.robot_command import RobotCommand

    ball_replacement = grSimPacketFactory.ball_replacement_command(x=0,y=0,vx=1)
    print(ball_replacement)
    
    robot_replacement = grSimPacketFactory.robot_replacement_command(x=0,
                            y=0,orientation=0,robot_id=1,isYellow=True)
    print(robot_replacement)
    cmd = RobotCommand(1,kick=True)
    robot_command = grSimPacketFactory.robot_command(**cmd.to_dict())
    print(robot_command)
