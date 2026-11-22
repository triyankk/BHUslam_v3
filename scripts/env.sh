#!/usr/bin/env bash
# Source this file from bash. All build tools and libraries stay in this workspace.
BHU_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export BHU_ROOT
export PATH="$BHU_ROOT/third_party/sysroot/usr/bin:$PATH"
export LD_LIBRARY_PATH="$BHU_ROOT/third_party/sysroot/usr/lib/aarch64-linux-gnu:$BHU_ROOT/third_party/orbbec/lib:$BHU_ROOT/build/pangolin:$BHU_ROOT/build/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_PREFIX_PATH="$BHU_ROOT/third_party/sysroot/usr:$BHU_ROOT/build/pangolin:${CMAKE_PREFIX_PATH:-}"
