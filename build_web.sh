#!/usr/bin/env bash
# Build the boids flocking simulation for WebAssembly via Emscripten.
#
# Requirements (all set up by this script when run once):
#   - emsdk (cloned into ./emsdk, latest SDK activated)
#   - raylib built for PLATFORM=Web into ./third_party/raylib-web/raylib
#
# Outputs go into ./build/web/ (boids.html + boids.js + boids.wasm).
set -euo pipefail

cd "$(dirname "$0")"

# --- emscripten toolchain ----------------------------------------------------
EMSDK_DIR="$PWD/emsdk"
EMSDK_ENV="$EMSDK_DIR/emsdk_env.sh"

if [ ! -f "$EMSDK_ENV" ]; then
  echo "emsdk not found at $EMSDK_DIR — cloning it now."
  git clone --depth 1 https://github.com/emscripten-core/emsdk.git "$EMSDK_DIR"
fi

# shellcheck source=/dev/null
source "$EMSDK_ENV"

echo "--- emcc: $(command -v emcc) ($(emcc --version | head -1))"

# --- build raylib for WebAssembly if not already present ---------------------
RAYLIB_WEB_DIR="$PWD/third_party/raylib-web/raylib"
EMSCRIPTEN_CMAKE="$EMSDK_DIR/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake"
if [ ! -f "$RAYLIB_WEB_DIR/libraylib.a" ] || [ ! -f "$RAYLIB_WEB_DIR/include/raylib.h" ]; then
  echo "--- building raylib for PLATFORM=Web"
  cmake -S third_party/raylib-src -B third_party/raylib-web \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$PWD/third_party/raylib-web" \
        -DPLATFORM=Web \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_SHARED_LIBS=OFF \
        -DCMAKE_TOOLCHAIN_FILE="$EMSCRIPTEN_CMAKE" \
        >/dev/null
  cmake --build third_party/raylib-web -j"$(nproc)" >/dev/null
fi

# --- configure & build the simulation for web --------------------------------
WEB_BUILD_DIR="build/web"
mkdir -p "$WEB_BUILD_DIR"

echo "--- configuring web build in $WEB_BUILD_DIR"
cmake -S . -B "$WEB_BUILD_DIR" \
      -DCMAKE_BUILD_TYPE=Release \
      -DBOIDS_WEB=ON \
      -DCMAKE_TOOLCHAIN_FILE="$EMSCRIPTEN_CMAKE" \
      >/dev/null

echo "--- building web build"
cmake --build "$WEB_BUILD_DIR" -j"$(nproc)"

echo
echo "Web build complete. Output in $WEB_BUILD_DIR/:"
ls -1h "$WEB_BUILD_DIR"/boids.{html,js,wasm} 2>/dev/null || true
echo
echo "Serve it with any static file server, e.g.:"
echo "  cd $WEB_BUILD_DIR && python3 -m http.server 8000"
echo "  then open http://localhost:8000/boids.html"
