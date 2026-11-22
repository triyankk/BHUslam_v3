#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
sdk=/home/atas/vscode/optFlow_slam/third_party/HesaiLidar_SDK_2.0
libs=/home/atas/vscode/optFlow_slam/build/jt16_bridge/hesai_sdk/libhesai
mkdir -p build/jt16-offline
g++ -std=c++17 -O2 -pthread tools/jt16/decode_raw.cpp \
  -I "$sdk/libhesai" -I "$sdk/libhesai/Common/include" \
  -I "$sdk/libhesai/UdpParser/include" -I "$sdk/libhesai/UdpProtocol" \
  -I "$sdk/libhesai/Logger/include" \
  "$libs/Common/libplatutils_lib.a" "$libs/Logger/liblog_lib.a" \
  -o build/jt16-offline/jt16_decode_raw
