#!/usr/bin/env bash
# Build (if needed) and run the boids simulation.
set -euo pipefail
cd "$(dirname "$0")"
if [ ! -f build/boids ]; then
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build build -j"$(nproc)"
fi
exec env LD_LIBRARY_PATH="$PWD/third_party/raylib/lib:${LD_LIBRARY_PATH:-}" ./build/boids "$@"
