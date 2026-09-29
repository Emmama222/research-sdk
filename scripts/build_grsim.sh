#!/usr/bin/env bash
# Ubuntu/WSL build of official grSim. No changes to the ODE physics implementation.
set -euo pipefail
cd "$(dirname "$0")/.."
repo_dir="$PWD"
source_dir="$repo_dir/.local/grsim-source"
venv_dir="$repo_dir/.local/physics-venv"
grsim_revision=fe2bd2915a46f9f11ea6cb48dc426b8047952073

if ! pkg-config --exists Qt5Widgets Qt5OpenGL ode protobuf; then
    echo 'Missing build dependencies. On Ubuntu/WSL run:'
    echo 'sudo apt-get install build-essential cmake pkg-config qtbase5-dev libqt5opengl5-dev libgl1-mesa-dev libglu1-mesa-dev libprotobuf-dev protobuf-compiler libode-dev libboost-dev python3-venv xvfb xauth'
    exit 1
fi
if [ ! -d "$source_dir/.git" ]; then
    git clone https://github.com/RoboCup-SSL/grSim.git "$source_dir"
    git -C "$source_dir" checkout --detach "$grsim_revision"
fi
if [ "$(git -C "$source_dir" rev-parse HEAD)" != "$grsim_revision" ]; then
    echo "Existing grSim checkout must be at $grsim_revision; refusing to overwrite it."
    exit 1
fi
if git -C "$source_dir" apply --check --unidiff-zero "$repo_dir/scripts/grsim-isolated-config.patch"; then
    git -C "$source_dir" apply --unidiff-zero "$repo_dir/scripts/grsim-isolated-config.patch"
else
    git -C "$source_dir" apply --reverse --check --unidiff-zero "$repo_dir/scripts/grsim-isolated-config.patch"
fi
if [ ! -x "$venv_dir/bin/python" ]; then
    python3 -m venv "$venv_dir"
fi
"$venv_dir/bin/python" -m pip install cmake==3.31.10 numpy networkx PyYAML protobuf pytest
"$venv_dir/bin/python" -m pip install -e . --no-deps
"$venv_dir/bin/cmake" -S "$source_dir" -B "$repo_dir/.local/grsim-build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_CLIENTS=OFF \
    -DCMAKE_INSTALL_PREFIX="$repo_dir/.local/grsim"
"$venv_dir/bin/cmake" --build "$repo_dir/.local/grsim-build" --parallel 4
"$venv_dir/bin/cmake" --install "$repo_dir/.local/grsim-build"
"$venv_dir/bin/python" scripts/record_grsim_build.py
