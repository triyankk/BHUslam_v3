#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$BHU_ROOT"
vins_deps="$BHU_ROOT/third_party/vins-deps"
if [[ ! -d third_party/VINS-Mono/vins_estimator || ! -d third_party/ceres-1.14/internal ]]; then
  echo 'Missing pinned VINS-Mono/Ceres sources; see docs/vins-mono-benchmark.md.' >&2
  exit 1
fi
export CMAKE_PREFIX_PATH="$vins_deps/sysroot/usr:$CMAKE_PREFIX_PATH"
cmake -S third_party/ceres-1.14 -B build/ceres-vins \
  -DCMAKE_INSTALL_PREFIX="$vins_deps/install" -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DSUITESPARSE=OFF -DCXSPARSE=OFF \
  -DLAPACK=OFF -DEIGENSPARSE=ON -DSCHUR_SPECIALIZATIONS=OFF \
  -DGFLAGS=OFF -DMINIGLOG=OFF -DOPENMP=OFF
cmake --build build/ceres-vins --parallel 1
cmake --install build/ceres-vins
cmake -S tools/vins_mono -B build/vins-mono -DCMAKE_BUILD_TYPE=Release
cmake --build build/vins-mono --parallel 1
