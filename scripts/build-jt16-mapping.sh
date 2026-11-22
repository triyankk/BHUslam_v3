#!/usr/bin/env bash
set -euo pipefail
jt16_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$jt16_root/scripts/env.sh"
source /home/atas/vscode/optFlow_slam/scripts/lio_env.sh
export CMAKE_PREFIX_PATH="/home/atas/vscode/optFlow_slam/runtime/lio/sysroot/usr:$CMAKE_PREFIX_PATH"
cmake -S "$jt16_root/tools/jt16" -B "$jt16_root/build/jt16" -DBHU_JT16_MAPPING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$jt16_root/build/jt16" --parallel 1
