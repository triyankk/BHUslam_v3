#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
source scripts/env.sh
cmake -S tools/livo -B build/livo-offline -DCMAKE_BUILD_TYPE=Release
cmake --build build/livo-offline --parallel 1
