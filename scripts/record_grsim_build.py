"""Record the executable, upstream source and configuration actually built."""

import hashlib
import json
import platform
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]
prefix = root / ".local/grsim"
binary = prefix / "bin/grSim"
source = root / ".local/grsim-source"
manifest = {
    "upstream": "https://github.com/RoboCup-SSL/grSim",
    "revision": subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip(),
    "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
    "robot_model_sha256": hashlib.sha256(
        (prefix / "share/grSim/config/Parsian.ini").read_bytes()
    ).hexdigest(),
    "platform": platform.platform(),
    "patch_sha256": hashlib.sha256(
        (root / "scripts/grsim-isolated-config.patch").read_bytes()
    ).hexdigest(),
    "source_diff": subprocess.check_output(
        ["git", "-C", str(source), "diff", "--ignore-space-at-eol"], text=True
    ),
    "ode_version": subprocess.check_output(
        ["pkg-config", "--modversion", "ode"], text=True
    ).strip(),
    "linked_libraries": subprocess.check_output(["ldd", str(binary)], text=True),
    "isolated_config": True,
}
(prefix / "build.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
print(f"Physics engine ready: {binary}")
