#!/usr/bin/env python
"""
Startup verification for TeamControl.

Run before main.py to confirm the environment is ready.
Each check is independent — all run even if earlier ones fail.
Exit code: 0 = all required checks passed, 1 = one or more failed.
"""

import socket
import struct
import sys
import time
from multiprocessing import Event, Process, Queue

TIMEOUT_VISION = 3.0  # seconds to wait for a vision packet
TIMEOUT_GC = 3.0  # seconds to wait for a GC packet (warning only)


# ── Helpers ───────────────────────────────────────────────────────────────────

_PASS = "[PASS]"
_FAIL = "[FAIL]"
_WARN = "[WARN]"
_SKIP = "[SKIP]"


def _ok(msg):
    print(f"  {_PASS}  {msg}")


def _fail(msg):
    print(f"  {_FAIL}  {msg}")


def _warn(msg):
    print(f"  {_WARN}  {msg}")


def _skip(msg):
    print(f"  {_SKIP}  {msg}")


def _header(title):
    print(f"\n{'─' * 55}")
    print(f"  {title}")
    print(f"{'─' * 55}")


# ── Check 1: Config file ──────────────────────────────────────────────────────


def check_config():
    _header("1. Config file (ipconfig.yaml)")
    try:
        from TeamControl.utils.yaml_config import Config

        cfg = Config()
        _ok(f"Parsed successfully")
        _ok(f"us_yellow={cfg.us_yellow}  us_positive={cfg.us_positive}")
        _ok(
            f"send_to_grSim={cfg.send_to_grSim}  use_grSim_vision={cfg.use_grSim_vision}"
        )
        _ok(f"Vision multicast: {cfg.vision[0]}:{cfg.vision[1]}")
        _ok(f"GC multicast:     {cfg.game_controller[0]}:{cfg.game_controller[1]}")
        _ok(f"robot_ip (bind):  {cfg.robot_ip}")
        return cfg
    except FileNotFoundError:
        _fail(
            "ipconfig.yaml not found — expected at src/TeamControl/utils/ipconfig.yaml"
        )
    except Exception as e:
        _fail(f"Parse error: {e}")
    return None


# ── Check 2: Protobuf imports ─────────────────────────────────────────────────


def check_protobufs():
    _header("2. Protobuf generated code")
    protos = [
        ("ssl_vision_wrapper_pb2", "TeamControl.network.proto2.ssl_vision_wrapper_pb2"),
        (
            "ssl_gc_referee_message_pb2",
            "TeamControl.network.proto2.ssl_gc_referee_message_pb2",
        ),
        ("grSim_Packet_pb2", "TeamControl.network.proto2.grSim_Packet_pb2"),
    ]
    all_ok = True
    for name, module in protos:
        try:
            __import__(module)
            _ok(name)
        except ImportError as e:
            _fail(f"{name}: {e}")
            all_ok = False
    if not all_ok:
        print(
            "    Fix: run 'python -m grpc_tools.protoc ...' or check that proto2/ .py files exist"
        )
    return all_ok


# ── Check 3: Vision multicast reachable ───────────────────────────────────────


def _recv_multicast(group, port, timeout, result_q):
    """Worker that joins multicast and tries to receive one packet."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(timeout)
        sock.bind(("", port))
        mreq = struct.pack("4sL", socket.inet_aton(group), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        data, addr = sock.recvfrom(6000)
        result_q.put(("ok", len(data), addr))
    except socket.timeout:
        result_q.put(("timeout", None, None))
    except Exception as e:
        result_q.put(("error", str(e), None))
    finally:
        try:
            sock.close()
        except Exception:
            pass


def _check_multicast(label, group, port, timeout, required=True):
    q = Queue()
    p = Process(target=_recv_multicast, args=(group, port, timeout, q), daemon=True)
    p.start()
    p.join(timeout + 1.0)

    if not q.empty():
        status, val, addr = q.get()
        if status == "ok":
            _ok(f"Received {val} bytes from {addr}")
            return True
        elif status == "timeout":
            msg = f"No packet in {timeout}s — is the source running?"
            if required:
                _fail(msg)
            else:
                _warn(msg)
            return False
        else:
            _fail(f"Socket error: {val}")
            return False
    else:
        _fail("Check process hung — possible permission issue joining multicast group")
        p.terminate()
        return False


def check_vision(cfg):
    _header("3. Vision multicast")
    if cfg is None:
        _skip("Skipped (no config)")
        return False
    group, port = cfg.vision
    return _check_multicast("Vision", group, port, TIMEOUT_VISION, required=True)


def check_game_controller(cfg):
    _header("4. Game Controller multicast (optional)")
    if cfg is None:
        _skip("Skipped (no config)")
        return True
    group, port = cfg.game_controller
    return _check_multicast("GC", group, port, TIMEOUT_GC, required=False)


# ── Check 4: Robot telemetry bind ─────────────────────────────────────────────


def check_robot_recv_bind(cfg):
    _header("5. Robot telemetry UDP bind")
    if cfg is None:
        _skip("Skipped (no config)")
        return False
    ip = cfg.robot_ip
    port = 50513
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((ip, port))
        sock.close()
        _ok(f"Bound to {ip}:{port}")
        return True
    except OSError as e:
        _fail(f"Cannot bind {ip}:{port}: {e}")
        print(
            f"    Fix: check that '{ip}' is the correct interface IP in ipconfig.yaml → network.robot_ip"
        )
        return False


# ── Check 5: grSim reachable (send test packet) ───────────────────────────────


def check_grsim(cfg):
    _header("6. grSim UDP send")
    if cfg is None:
        _skip("Skipped (no config)")
        return True
    if not cfg.send_to_grSim:
        _skip("send_to_grSim=false in config — skipped")
        return True

    ip, port = cfg.grSim_addr
    try:
        from TeamControl.network.grSimPacketFactory import grSimPacketFactory
        from TeamControl.network.proto2 import grSim_Packet_pb2

        packet = grSimPacketFactory.robot_command(
            robot_id=0, vx=0, vy=0, w=0, kick=0, dribble=0, isYellow=True
        )
        data = packet.SerializeToString()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(data, (ip, port))
        sock.close()
        _ok(f"Sent {len(data)}-byte test packet to {ip}:{port}")
        return True
    except Exception as e:
        _warn(
            f"Send to grSim failed: {e}  (grSim may not be running — non-fatal if not in grSim mode)"
        )
        return True


# ── Check 6: WorldModel manager IPC sanity ────────────────────────────────────


def _wm_worker(result_q):
    try:
        from TeamControl.world.model_manager import WorldModelManager

        mgr = WorldModelManager()
        mgr.start()
        wm = mgr.WorldModel()
        v = wm.get_version()
        result_q.put(("ok", v))
        mgr.shutdown()
    except Exception as e:
        result_q.put(("error", str(e)))


# def check_world_model():
#     _header("7. WorldModel manager (IPC proxy)")
#     q = Queue()
#     p = Process(target=_wm_worker, args=(q,), daemon=True)
#     p.start()
#     p.join(5.0)

#     if not q.empty():
#         status, val = q.get()
#         if status == "ok":
#             _ok(f"Manager started, proxy call returned version={val}")
#             return True
#         else:
#             _fail(f"Manager error: {val}")
#             return False
#     else:
#         _fail("WorldModel manager process hung (timeout 5s)")
#         p.terminate()
#         return False


# ── Check 7: Core imports ─────────────────────────────────────────────────────


def check_imports():
    _header("8. Core module imports")
    modules = [
        "TeamControl.process_workers.vision_runner",
        "TeamControl.process_workers.gcfsm_runner",
        "TeamControl.process_workers.wm_runner",
        "TeamControl.process_workers.robot_recv_runner",
        "TeamControl.dispatcher.dispatch",
        "TeamControl.robot.striker",
        "TeamControl.robot.goalie",
        "TeamControl.robot.team",
    ]
    all_ok = True
    for m in modules:
        try:
            __import__(m)
            _ok(m.split(".")[-1])
        except Exception as e:
            _fail(f"{m}: {e}")
            all_ok = False
    return all_ok


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    print("\n" + "=" * 55)
    print("  TeamControl Startup Verification")
    print("=" * 55)

    results = {}

    cfg = check_config()
    results["config"] = cfg is not None
    results["protobufs"] = check_protobufs()
    results["imports"] = check_imports()
    results["vision"] = check_vision(cfg)
    results["gc"] = check_game_controller(cfg)  # warning only
    results["robot_bind"] = check_robot_recv_bind(cfg)
    results["grsim"] = check_grsim(cfg)
    # results["worldmodel"] = check_world_model()

    # ── Summary ───────────────────────────────────────────────────────────────
    print("\n" + "=" * 55)
    print("  Summary")
    print("=" * 55)
    required = ["config", "protobufs", "imports", "vision", "robot_bind","gc"]
    optional = [ "grsim"] #optional : "worldmodel"

    for k in required:
        status = _PASS if results[k] else _FAIL
        print(f"  {status}  {k}")
    for k in optional:
        status = _PASS if results[k] else _WARN
        print(f"  {status}  {k} (optional)")

    failed_required = [k for k in required if not results[k]]
    if failed_required:
        print(
            f"\n  {len(failed_required)} required check(s) failed: {', '.join(failed_required)}"
        )
        print("  Fix the issues above before running main.py\n")
        sys.exit(1)
    else:
        print("\n  All required checks passed — safe to run main.py\n")
        sys.exit(0)


if __name__ == "__main__":
    main()
