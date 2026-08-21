from research_sdk.vision.frame import Frame
from research_sdk.vision.field import GeometryData
from research_sdk.network.ssl_sockets import Vision,VisionTracker
from research_sdk.config import SSL_VISION_PORT

from research_sdk.process_workers.worker import BaseWorker

import numpy as np
import time



class VisionProcess(BaseWorker):
    GRSIM_CAMERAS = 4
    REAL_LIFE_CAMERAS = 1
    
    def __init__(self,is_running,logger):
        super().__init__(is_running=is_running,logger=logger)
        self.loop_timer = time.time()
        self.field = None
        self.frame = None
        self.frame_number = -1
        self.error_loop_count =0 
        
    @property
    def cameras(self):
        return self.GRSIM_CAMERAS if self.use_grSim is True else self.REAL_LIFE_CAMERAS
    
    @property
    def has_field(self):
        return self.field is not None
    
    def setup(self,*args):
        output_q, use_grSim, vision_port = args
        self.use_grSim = use_grSim
        self.output_q = output_q
        self.recv = Vision(is_running=self.is_running,port=vision_port)    
        self.logger.info(f"[VP] : now listening on {vision_port}, using grSim ? {use_grSim} cameras : {self.cameras}")


    def step(self) -> None:
        # listen for data
        new_vision_data = self.recv.listen()
        # if after timeout it is none
        if new_vision_data is None:
            self.logger.warning("[VP] : No vision data received")
            return # skip this loop
                
        # if we received the vision data and it has type Detection: 
        if new_vision_data.HasField("detection"):
            self.update_detection(new_vision_data.detection)
                
        if new_vision_data.HasField("geometry"):
            self.update_geometry(new_vision_data.geometry)
            
        self.loop_timer = time.time()
    
    def update_geometry(self,new_geometry):
        # replace variable for simplicity
        self.field = GeometryData.from_proto(new_geometry)
        self.logger.debug(f"[VP] : frame: {self.frame_number} has geometry")
        self.send(self.field)
    
    def update_detection(self,new_detection_data):
        # if this frame number is older than the new frame
        if self.frame_number < new_detection_data.frame_number:
            # generates new frame
            self.frame_number = new_detection_data.frame_number
            self.frame = Frame.from_proto(new_detection_data,self.cameras)
            self.logger.debug(f"[VP] :We get new frame : {new_detection_data.frame_number}")


        # if same frame number = it is old frame
        elif self.frame_number == new_detection_data.frame_number:
            # frame already sent (duplicate camera packet after completion), ignore
            if self.frame is None:
                return
            # we update the original frame
            self.logger.debug(f"[VP] : Updating old frame : {new_detection_data.frame_number}")
            self.frame.update(new_detection_data)

        # if the frame is now completed
        if self.frame is not None and self.frame.is_completed is True:
            self.logger.debug(f"[VP] : frame: {self.frame_number} has been completed with {self.cameras} cameras , time taken = {time.time() - self.loop_timer}")
            self.send(self.frame)
            self.frame = None
            self.loop_timer = time.time()
    
    def send(self,data):
        self.logger.debug("Sending data")

        if not self.output_q.full():
            self.output_q.put(data)
        else:
            self.logger.warning("[VP] : VISION QUEUE IS FULL — frame dropped")


if __name__ == "__main__" :
    from multiprocessing import Queue, Process, Event,freeze_support
    import sys
    freeze_support()
    is_running = Event()
    is_running.set()

    def read(input_q):
        count = 0
        while True:
            try:
            
                if not input_q.empty():
                    item = input_q.get_nowait()
                    t = str(type(item))
                    # print(t)
                    if t == "<class 'TeamControl.vision.field.GeometryData'>":
                        count += 1
                        print(item)
                        if count == 4 :
                            is_running.clear()
                            break
                    # print(type(item))
                else:
                    time.sleep(1)
                
            except KeyboardInterrupt:
                print(f" Force Quitting")
                sys.exit()
    
    logger = None
    output_q, use_grSim, vision_port = Queue(), True, SSL_VISION_PORT

    vision = Process(target=VisionProcess.run_worker,args=(is_running,logger,output_q, use_grSim, vision_port))
    reader = Process(target=read,args=(output_q,))
    
    vision.start()
    reader.start()
    vision.join()
    reader.join()
