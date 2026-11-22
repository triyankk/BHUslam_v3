#!/usr/bin/env bash
set -euo pipefail
tof_cpp_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$tof_cpp_dir/build"
tof_build_tmp="$(mktemp "$tof_cpp_dir/build/tof_viewer.XXXXXX")"
trap 'rm -f -- "$tof_build_tmp"' EXIT
g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wpedantic \
  -isystem "$tof_cpp_dir/vendor" \
  -I "$tof_cpp_dir/../../../include" \
  -DTOF_ASSET_DIR="\"$tof_cpp_dir/..\"" \
  "$tof_cpp_dir/main.cpp" -lcrypto -o "$tof_build_tmp"
"$tof_build_tmp" --self-test
mv -f -- "$tof_build_tmp" "$tof_cpp_dir/build/tof_viewer"
