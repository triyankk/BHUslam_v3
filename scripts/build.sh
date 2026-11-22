#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$BHU_ROOT"
cmake -S third_party/Pangolin -B build/pangolin -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_TESTS=OFF \
  -DCMAKE_EXPORT_PACKAGE_REGISTRY=OFF
cmake --build build/pangolin --target pango_display pango_plot --parallel "${BHU_JOBS:-1}"
cmake -S . -B build/main -DCMAKE_BUILD_TYPE=Release -DBHU_BUILD_CAMERA=ON -DBHU_BUILD_SLAM=ON
cmake --build build/main --parallel "${BHU_JOBS:-1}"
ctest --test-dir build/main --output-on-failure
if [[ ! -f third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt ]]; then
  tar -xzf third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz -C third_party/ORB_SLAM3/Vocabulary
fi
