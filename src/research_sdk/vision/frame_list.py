from research_sdk.vision.frame import Frame
from collections import deque

import numpy as np
import numpy.typing as npt

class FrameList ():
    
    ### THIS IS A LIST CLASS so this would work like a list
    def __init__(self,history:int=60):
        self.newest_frame = 0
        self.history = history
        self._frames = deque(maxlen=history)
        self._frame_lookup = {}  # Maps frame_id -> Frame
    
    
    def __repr__(self):
        return repr(self._frames)
    

    @property
    def frame_ids(self) -> list[int]:
        return [frame.frame_number for frame in self._frames]    
    
    @property
    def latest(self) -> Frame | None:
        return self._frames[-1] if self._frames else None


        
    def append(self, frame: Frame):
        if frame.frame_number in self._frame_lookup:
            raise LookupError (f"{frame.frame_number} exist")
            self.update(frame)
        # If full, remove oldest from both deque and dict
        if len(self._frames) == self.history:
            old_frame = self._frames.popleft() 
            del self._frame_lookup[old_frame.frame_number] 
        # Add new frame
        self._frames.append(frame)
        self._frame_lookup[frame.frame_number] = frame # adds to dictionary

    def get_frame_withid(self, frame_id: int) -> Frame | None:
        return self._frame_lookup.get(frame_id)

    def get_last_n_frames(self, n: int) -> list[Frame]:
        if n >= len(self._frames):
            return list(self._frames)
        return [self._frames[-n + i] for i in range(n)]

    
    def clear(self):
        self._frames.clear()
    
    # * index is 0-100, not frame number 
    def __getitem__(self, index):
        return self._frames[index]

    def __setitem__(self, index, value):
        self._frames[index] = value

    def __len__(self):
        return len(self._frames)

    def __iter__(self):
        return iter(self._frames)