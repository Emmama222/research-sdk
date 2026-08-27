import time

from research_sdk.config import SSL_VISION_PORT
from research_sdk.network.ssl_sockets import Vision
from research_sdk.process_workers.worker import BaseWorker
from research_sdk.vision.field import GeometryData
from research_sdk.vision.frame import Frame


class VisionFrameAssembler:
    """Combine one detection cycle across independent camera streams.

    SSL camera frame numbers are not a shared cross-camera sequence. Camera IDs,
    rather than equal frame numbers, therefore define a complete cycle. Seeing a
    camera twice closes the previous partial cycle, which also supports grSim
    configurations that publish fewer cameras than expected.
    """

    def __init__(self, cameras: int) -> None:
        self.cameras = cameras
        self.frame: Frame | None = None
        self.frame_number = -1

    def push(self, detection) -> Frame | None:
        completed = None
        camera_id = int(detection.camera_id)
        if self.frame is None:
            self.frame = Frame.from_proto(detection, self.cameras)
        elif camera_id in self.frame.cameras:
            completed = self.frame
            self.frame = Frame.from_proto(detection, self.cameras)
        else:
            self.frame.update(detection)
        self.frame_number = int(detection.frame_number)
        if self.frame is not None and self.frame.is_completed:
            completed = self.frame
            self.frame = None
        return completed



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
        self.assembler = VisionFrameAssembler(self.cameras)
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
        frame = self.assembler.push(new_detection_data)
        self.frame_number = self.assembler.frame_number
        self.frame = self.assembler.frame
        if frame is not None:
            self.logger.debug(f"[VP] : frame: {self.frame_number} has been completed with {self.cameras} cameras , time taken = {time.time() - self.loop_timer}")
            self.send(frame)
            self.loop_timer = time.time()
    
    def send(self,data):
        self.logger.debug("Sending data")

        if not self.output_q.full():
            self.output_q.put(data)
        else:
            self.logger.warning("[VP] : VISION QUEUE IS FULL — frame dropped")


if __name__ == "__main__" :
    import sys
    from multiprocessing import Event, Process, Queue, freeze_support
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
                print(" Force Quitting")
                sys.exit()
    
    logger = None
    output_q, use_grSim, vision_port = Queue(), True, SSL_VISION_PORT

    vision = Process(target=VisionProcess.run_worker,args=(is_running,logger,output_q, use_grSim, vision_port))
    reader = Process(target=read,args=(output_q,))
    
    vision.start()
    reader.start()
    vision.join()
    reader.join()
