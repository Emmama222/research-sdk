"""Record the executable, upstream source and configuration actually built.

Writes <prefix>/build.json, which grsim_physics.GrSimSession refuses to run
without: it checks `isolated_config` and that the binary on disk still hashes
to `binary_sha256`, so a rebuilt or substituted executable cannot inherit an
old stamp.

build_grsim.sh calls this with no arguments for the Linux/WSL build under
.local/. For a native Windows build (MSYS2, docs/physics.md) point it at the
install prefix and the patched source checkout:

    python scripts/record_grsim_build.py --prefix %USERPROFILE%\\grsim\\install --source %USERPROFILE%\\grsim\\src

`ldd` and `pkg-config` are what the Linux build has. On Windows the same
facts come from `objdump -p` (the DLL import table) and `pkgconf` from the
MSYS2 toolchain; whichever tool answered is named in the stamp.
"""

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]


def first_available(commands: list[list[str]]) -> tuple[str, str]:
    """Output of the first command whose executable exists, with its name."""
    for command in commands:
        exe = shutil.which(command[0])
        if exe is None:
            continue
        result = subprocess.run([exe, *command[1:]], capture_output=True, text=True, check=False)
        if result.returncode == 0:
            return command[0], result.stdout
    return "", "unavailable: none of " + ", ".join(c[0] for c in commands)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prefix", type=Path, default=root / ".local/grsim")
    ap.add_argument("--source", type=Path, default=root / ".local/grsim-source")
    ap.add_argument("--binary", type=Path, default=None,
                    help="default <prefix>/bin/grSim, or grSim.exe on Windows")
    args = ap.parse_args()
    prefix = args.prefix.resolve()
    binary = args.binary or prefix / "bin" / ("grSim.exe" if os.name == "nt" else "grSim")
    source = args.source.resolve()
    if os.name == "nt":
        # objdump and pkgconf ship with the MSYS2 toolchain that built the
        # executable; put it in reach without requiring it on the user's PATH.
        os.environ["PATH"] = r"C:\msys64\mingw64\bin" + os.pathsep + os.environ["PATH"]
    libs_tool, libs = first_available([["ldd", str(binary)], ["objdump", "-p", str(binary)]])
    if libs_tool == "objdump":
        libs = "\n".join(line.strip() for line in libs.splitlines() if "DLL Name:" in line)
    ode_tool, ode = first_available([["pkg-config", "--modversion", "ode"], ["pkgconf", "--modversion", "ode"]])
    manifest = {
        "upstream": "https://github.com/RoboCup-SSL/grSim",
        "revision": subprocess.check_output(
            ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
        ).strip(),
        "binary": str(binary),
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
        "ode_version": ode.strip(),
        "ode_version_tool": ode_tool,
        "linked_libraries": libs,
        "linked_libraries_tool": libs_tool,
        "isolated_config": True,
    }
    (prefix / "build.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Physics engine ready: {binary}")


if __name__ == "__main__":
    main()
