#!/usr/bin/env bash
# Bounded bench test. No source selection, parameter changes or arming.
set -euo pipefail
test_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$test_workspace/scripts/env.sh"
if (( $# < 1 || $# > 3 )); then
  echo 'Usage: scripts/run-vins-fusion-latency.sh NEW_OUTPUT [SECONDS=60] [--cube]' >&2
  exit 2
fi
test_output="$(realpath -m -- "$1")"
test_seconds="${2:-60}"
test_cube="${3:-}"
[[ "$test_seconds" =~ ^[0-9]+$ ]] && (( test_seconds >= 1 && test_seconds <= 600 )) || { echo 'Seconds must be 1..600' >&2; exit 2; }
[[ -z "$test_cube" || "$test_cube" == --cube ]] || { echo 'Unknown option' >&2; exit 2; }
[[ ! -e "$test_output" ]] || { echo 'Output already exists' >&2; exit 1; }
test_profile="$(/usr/sbin/nvpmodel -q)"
[[ "$test_profile" == *'NV Power Mode: 15W'* ]] || { echo 'Select 15W first: bin/vio-test power-15w' >&2; exit 1; }
mkdir -p "$test_output"
printf '%s\n' "$test_profile" > "$test_output/power-before.txt"
test_bridge_pid=''
test_live_pid=''
test_stats_pid=''
cleanup_latency() {
  trap - EXIT INT TERM
  for test_pid in "$test_live_pid" "$test_bridge_pid" "$test_stats_pid"; do
    if [[ -n "$test_pid" ]]; then kill -TERM "$test_pid" 2>/dev/null || true; wait "$test_pid" 2>/dev/null || true; fi
  done
}
trap cleanup_latency EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
test_options=()
if [[ "$test_cube" == --cube ]]; then
  "$test_workspace/build/main/bin/cube_vio_bridge" --output "$test_output/bridge" --seconds "$((test_seconds+20))" --quality-min 40 --cue start > "$test_output/bridge.console.log" 2>&1 &
  test_bridge_pid=$!
  for ((test_attempt=0;test_attempt<100;test_attempt++)); do
    [[ -f "$test_output/bridge/ready.json" ]] && break
    kill -0 "$test_bridge_pid" 2>/dev/null || { cat "$test_output/bridge.console.log" >&2; exit 1; }
    sleep .1
  done
  [[ -f "$test_output/bridge/ready.json" ]] || { echo 'Cube link not ready' >&2; exit 1; }
  test_options=(--udp-port 14600)
fi
if [[ -x /usr/bin/tegrastats ]]; then
  /usr/bin/tegrastats --interval 1000 > "$test_output/tegrastats.log" 2>&1 &
  test_stats_pid=$!
fi
"$test_workspace/bin/vins-fusion-live" "$test_workspace/configs/gemini336l-CPC2963000JJ-fast.yaml" "$test_workspace/configs/vins-fusion-extnav.yaml" "$test_output/live" --seconds "$test_seconds" --allow-provisional "${test_options[@]}" > "$test_output/live.console.log" 2>&1 &
test_live_pid=$!
while kill -0 "$test_live_pid" 2>/dev/null; do
  if [[ -n "$test_bridge_pid" ]] && ! kill -0 "$test_bridge_pid" 2>/dev/null; then echo 'Cube bridge stopped; stopping live test' >&2; exit 1; fi
  sleep .2
done
wait "$test_live_pid"
test_live_pid=''
/usr/sbin/nvpmodel -q > "$test_output/power-after.txt"
sha256sum "$test_workspace/build/vins-fusion/bin/vins_fusion_live" "$test_workspace/build/main/bin/cube_vio_bridge" > "$test_output/binaries.sha256"
echo "Test complete: $test_output/live/summary.json"
