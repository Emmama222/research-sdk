# typings
from multiprocessing import Queue,Process,Event
from research-sdk.SSL.vision.field import GeometryData
from research-sdk.SSL.vision.frame import Frame
from research-sdk.world.model import WorldModel
from research-sdk.utils.Logger import LogSaver
from research-sdk.process_workers.worker import BaseWorker
from research-sdk.onboard_vision import parse_packet
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

        drained_gc = 0
        while not self.gc_q.empty() and drained_gc < 32:
            new_info = self.gc_q.get_nowait()
            drained_gc += 1
            self.logger.info(f"[wmr] : Updating World Model Game Info {new_info[0]}")
            self.wm.update_gc_data(new_info)
            from research-sdk.SSL.game_controller.common import PacketType
            if new_info[0] == PacketType.NEW_STATE:
                print(f"[wmr] → new game state: {new_info[1]}", flush=True)
            self.wm.update_gc_data(new_info)

        if self.recv_q is not None:
            drained = 0
            while drained < 32:
                try:
                    data, addr = self.recv_q.get_nowait()
                except Exception:
                    break
                drained += 1
                obs = parse_packet(data)
                if obs is None:
                    self._onboard_rejected += 1
                    self.logger.warning(
                        f"[wmr] onboard: parse_packet returned None "
                        f"from {addr} len={len(data)} "
                        f"head={data[:60]!r}")
                    continue
                obs.recv_ts = time.time()
                if obs.robot_id < 0 and addr:
                    m = self.ip_map.get(addr[0])
                    if m is not None:
                        obs.is_yellow = bool(m[0])
                        obs.robot_id = int(m[1])
                if obs.robot_id < 0:
                    self._onboard_rejected += 1
                    self.logger.warning(
                        f"[wmr] onboard: no robot_id for {addr} "
                        f"(ip_map has {len(self.ip_map)} entries)")
                    continue
                self.wm.put_onboard_obs(obs)
                self._onboard_ingested += 1

            now = time.time()
            if drained and (now - self._last_onboard_log) > 2.0:
                self._last_onboard_log = now
                msg = (f"[wmr] onboard totals: ingested="
                       f"{self._onboard_ingested} "
                       f"rejected={self._onboard_rejected}")
                self.logger.info(msg)
                print(msg, flush=True)

        time.sleep(self.delay_time)
    
    def run(self):
        return super().run()   
    
    def shutdown(self):
        return super().shutdown()        

if __name__ == "__main__":
    logger = LogSaver()
    is_running = Event()
    is_running.set()
    
    wm = WorldModel()
    gc_q = Queue()
    vision_q = Queue()
    
    worker = Process(target=WMWorker.run_worker,args=(is_running,logger,wm,vision_q,gc_q,),)
    worker.start()
    try: 
        print("[main] : type something to quit")
        s = input()
        print("[main] : finishing this loop")
        is_running.clear()
        
    
    except KeyboardInterrupt:
        logger.info(f"[main] : Force Quitting workers ")
        is_running.clear()

    logger.info("[main] : waiting for workers to be shut down")
    worker.join(timeout=4)
