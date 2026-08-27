# typings
from multiprocessing import Queue,Process,Event
from research_sdk.vision.field import GeometryData
from research_sdk.vision.frame import Frame
from research_sdk.world.model import WorldModel
from research_sdk.process_workers.worker import BaseWorker
# from research_sdk.onboard_vision import parse_packet
import logging
import time


class WMWorker(BaseWorker):
    def __init__(self,is_running,logger):
        super().__init__(is_running=is_running,logger=logger)
        self.delay_time = 0.001 # s
        self.recv_q: Queue | None = None
        self.ip_map: dict = {}
        self._onboard_ingested = 0
        self._onboard_rejected = 0
        self._last_onboard_log = 0.0


    def setup(self, *args):
        """ setup for wm :
        expected in order :
            wm       = world model shared object
            vision_q = Queue from vision
            gc_q     = Queue from gcfsm
            recv_q   = Queue from RobotRecv (optional)
            ip_map   = dict ip -> (is_yellow, robot_id) (optional)
        """
        if len(args) >= 5:
            wm, vision_q, gc_q, recv_q, ip_map = args[:5]
        elif len(args) == 4:
            wm, vision_q, gc_q, recv_q = args
            ip_map = {}
        else:
            wm, vision_q, gc_q = args
            recv_q, ip_map = None, {}

        self.wm:WorldModel = wm
        self.vision_q:Queue = vision_q
        self.gc_q:Queue = gc_q
        self.recv_q = recv_q
        self.ip_map = ip_map or {}
        self.logger.info(
            f"[wmr] : L setup completed (recv_q={'on' if recv_q else 'off'}, "
            f"ip_map={len(self.ip_map)} entries)")

    def step(self):
        if not self.vision_q.empty() :
            item = self.vision_q.get()
            if isinstance(item,Frame):
                self.logger.info("[wmr] : Updating World Model Frame")
                self.wm.add_new_frame(item)
            elif isinstance(item,GeometryData):
                self.logger.info("[wmr] : Updating World Model Geometry")
                self.wm.update_geometry(item)

    def run(self):
        return super().run()   
    
    def shutdown(self):
        return super().shutdown()        

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("research_sdk.worker.world_model")
    is_running = Event()
    is_running.set()
    
    wm = WorldModel()
    gc_q = Queue()
    vision_q = Queue()
    
    worker = Process(target=WMWorker.run_worker,args=(is_running,logger,wm,vision_q,gc_q,),)
    worker.start()
    try: 
        input("Press Enter to quit\n")
        logger.info("stopping world-model worker")
        is_running.clear()
        
    
    except KeyboardInterrupt:
        logger.info(f"[main] : Force Quitting workers ")
        is_running.clear()

    logger.info("[main] : waiting for workers to be shut down")
    worker.join(timeout=4)
