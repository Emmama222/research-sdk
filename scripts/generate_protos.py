"""Regenerate Python protobuf bindings from src/research_sdk/proto/*.proto.

Run whenever the .proto files change (e.g. after updating grSim or the SSL
simulation protocol version):

    python scripts/generate_protos.py

Requires the `dev` extra: pip install -e ".[dev]"
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PROTO_DIR = Path(__file__).resolve().parent.parent / "src" / "research_sdk" / "proto"
OUT_DIR = PROTO_DIR / "generated"


def main() -> None:
    OUT_DIR.mkdir(exist_ok=True)
    proto_files = sorted(str(p) for p in PROTO_DIR.glob("*.proto"))
    if not proto_files:
        raise SystemExit(f"No .proto files found in {PROTO_DIR}")

    subprocess.run(
        [
            sys.executable,
            "-m",
            "grpc_tools.protoc",
            f"-I={PROTO_DIR}",
            f"--python_out={OUT_DIR}",
            *proto_files,
        ],
        check=True,
    )
    print(f"Generated bindings for {len(proto_files)} .proto files into {OUT_DIR}")


if __name__ == "__main__":
    main()
