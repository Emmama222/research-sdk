import struct
import time

class RobotCommands:
    FORMAT = '<Ifff??d'
    SIZE = struct.calcsize(FORMAT)

    def __init__(self, robot_id:int, vx:float, vy:float, w:float, kick:bool, dribble:bool, time_origin:float=None):
        """RobotCommands

        Args:
            robot_id (int): Robot ID
            vx (float): Velocity in X direction of Robot
            vy (float): Velocity in Y direction of Robot
            w (float): Angular Velocity of Robot
            kick (bool): Robot Kick ? 
            dribble (bool): Robot Dribble ?
            time_origin (float, optional): Time of RobotCommand being generated. Defaults to None.
        """
        self.robot_id = robot_id
        self.vx = vx
        self.vy = vy
        self.w = w
        self.k = kick
        self.d = dribble
        self.time_created = time.time()
        self.time_origin = time_origin
    
    def pack(self):
        return struct.pack(self.FORMAT, self.robot_id, self.vx, self.vy, self.w, self.k, self.d, self.time_created)

    @classmethod
    def unpack(cls, data):
        if len(data) != cls.SIZE:
            raise ValueError("Invalid data size")
        robot_id, vx, vy, w, kick, dribble, time_origin= struct.unpack(cls.FORMAT, data)
        return cls(robot_id, vx, vy, w, kick, dribble, time_origin)

    def __repr__(self):
        return f"<Status: robot={self.robot_id}. Velocity_X(vx)={self.vx}, Velocity_Y(vy)={self.vy}, Angular_Velocity(w)={self.w}, kick?={self.k}, dribble?={self.d} time_created={self.time_created}, time_origin={self.time_origin}>"

if __name__ == "__main__":
    packet = RobotCommands(robot_id=1,vx=1.1,vy=1.1,w=2.2,kick=0,dribble=1)
    print(packet)
    packet_as_struct = packet.pack()
    print(packet_as_struct)
    unpack = RobotCommands.unpack(packet_as_struct)
    print(unpack)