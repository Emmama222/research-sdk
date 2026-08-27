"""Base class for long-running process workers."""

import logging
from multiprocessing import Event, Process
import time


class BaseWorker:
    def __init__(self, is_running, logger: logging.Logger | None = None):
        self.is_running = is_running
        self.logger = logger or logging.getLogger(
            f"research_sdk.worker.{self.__class__.__name__}"
        )
        self.error_cnt = 0
        self.last_error_time = 0
        
    def setup(self, *args):
        """
        This is where you parse in other variables to continue the setup
        e.g. world model, queues
        """
        time.sleep(1)
        self.logger.info("setup complete")
        
    def step(self):
        """
        each step in the loop
        replace this for a more functional code :) 
        
        """
        self.logger.debug("working")
        time.sleep(1)
    
    def run(self):
        while self.is_running.is_set():
            try:
                self.step()
            except KeyboardInterrupt:
                self.logger.warning("interrupted")
                break
            except Exception:
                self.logger.exception("worker step failed")
                self.error_cnt += 1

                now = time.time()
                if now - self.last_error_time > 4 or self.last_error_time == 0:
                    self.last_error_time = now
                    self.error_cnt = 1
                
                if self.error_cnt >= 4:
                    self.error_cnt = 0
                    self.logger.error("stopping after repeated worker errors")
                    break
        
        self.shutdown()
    
    # do shutdown here 
    def shutdown(self):
        self.logger.info("shutting down")
        time.sleep(0.1)
        self.logger.info("offline")

    @classmethod
    def run_worker(cls, is_running, logger, *args):
        """
        the Multiprocessing Process initiator

        Args:
            worker (BaseWorker): Any worker that is a subclass of this
            is_running (Event): The main Event that controls the running state of the system
            args(*args) : other optional arguments for setting up 
        """
        w = cls(is_running, logger)
        w.setup(*args)
        w.run()
        
    

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("research_sdk.worker.demo")
    is_running = Event()
    is_running.set()
    worker = Process(target=BaseWorker.run_worker, args=(is_running, logger))
    worker.start()
    try:
        input("Press Enter to quit\n")
        logger.info("stopping demo worker")
    except KeyboardInterrupt:
        logger.info("interrupting demo worker")
    finally:
        is_running.clear()

    logger.info("waiting for worker shutdown")
    worker.join(timeout=4)
    logger.info("all workers offline")
