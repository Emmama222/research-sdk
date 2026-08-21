
import numpy as np

class Ball:
    __slots__ = ('c', 'x', 'y', 'px', 'py', '_position')

    def __init__(self,ball_data)->None:
        self.c = float(ball_data.confidence)
        self.x = round(float(ball_data.x), 4)
        self.y = round(float(ball_data.y), 4)
        self.px = round(float(ball_data.pixel_x), 4)
        self.py = round(float(ball_data.pixel_y), 4)
        self._position: tuple[float,float]|None = None


    def __repr__(self):
        return f"BALL \n Confidence : {self.c:.4f}\n POSITION : {self.position} \n PIXEL : {self.px:.4f}, {self.py:.4f}\n"

    @property
    def position (self) -> tuple[float,float]:
        if self._position is None:
            self._position = (self.x,self.y)
        return self._position

