#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$BHU_ROOT"
export CMAKE_PREFIX_PATH="$BHU_ROOT/third_party/vins-deps/sysroot/usr:$CMAKE_PREFIX_PATH"
cmake -S tools/vins_fusion -B build/vins-fusion -DCMAKE_BUILD_TYPE=Release
cmake --build build/vins-fusion --parallel 1
