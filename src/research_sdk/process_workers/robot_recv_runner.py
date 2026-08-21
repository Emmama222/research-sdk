import socket
from multiprocessing import Queue

from research-sdk.process_workers.worker import BaseWorker

RECV_PORT = 50513
RECV_TIMEOUT = 0.5
BUFFER_SIZE = 2048


class RobotRecv(BaseWorker):
    """UDP listener for per-robot onboard telemetry.

    Binds directly to 0.0.0.0:50513 and forwards raw bytes into recv_q.
    We deliberately bypass `network.receiver.Receiver` because its
    `_decode` does `data.decode("utf-8")` — any non-UTF-8 byte raises
    UnicodeDecodeError out of `listen()`, which BaseWorker counts as a
    worker error and kills the process after 4 in a row. `parse_packet`
    (downstream in WMWorker) already accepts both bytes and str.
    """

    def __init__(self, is_running, logger):
        super().__init__(is_running, logger)
        self._queue: Queue | None = None
        self._sock: socket.socket | None = None
        self._packets = 0
        self._parse_errors = 0
        self._last_log = 0.0

    def setup(self, *args):
        robot_ip, recv_queue = args
        self._queue = recv_queue
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((robot_ip, RECV_PORT))
        sock.settimeout(RECV_TIMEOUT)
        self._sock = sock
        msg = f"[RobotRecv] listening on {robot_ip},:{RECV_PORT}"
        self.logger.info(msg)
        print(msg, flush=True)
        super().setup()

    def step(self):
        if self._sock is None or self._queue is None:
            return
        try:
            data, addr = self._sock.recvfrom(BUFFER_SIZE)
        except socket.timeout:
            return
        except OSError as e:
            self.logger.error(f"[RobotRecv] socket error: {e}")
            return

        self._queue.put((data, addr))
        self._packets += 1

        import time as _t

        now = _t.time()
        if self._packets <= 5 or (now - self._last_log) > 2.0:
            self._last_log = now
            preview = (
                data[:60] if isinstance(data, (bytes, bytearray)) else str(data)[:60]
            )
            msg = (
                f"[RobotRecv] pkt #{self._packets} from {addr} "
                f"len={len(data)} head={preview!r}"
            )
            self.logger.info(msg)
            print(msg, flush=True)

    def shutdown(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        super().shutdown()
