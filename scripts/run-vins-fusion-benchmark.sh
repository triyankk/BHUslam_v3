#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$BHU_ROOT"
bench_out="${1:?Usage: run-vins-fusion-benchmark.sh NEW_OUTPUT}"
[[ ! -e "$bench_out" ]] || { echo 'Output exists' >&2;exit 1; }
bench_power="$(/usr/sbin/nvpmodel -q)"
[[ "$bench_power" == *'NV Power Mode: 15W'* ]] || { echo "Expected 15W; actual: $bench_power" >&2;exit 1; }
mkdir -p "$bench_out"
printf '%s\n' "$bench_power" > "$bench_out/power-start.txt"
cat /proc/sys/kernel/random/boot_id > "$bench_out/boot-id.txt"
git -C third_party/VINS-Fusion rev-parse HEAD > "$bench_out/fusion-commit.txt"
git -C third_party/VINS-Fusion diff --exit-code > "$bench_out/upstream.diff"
sha256sum build/vins-fusion/bin/vins_fusion_replay > "$bench_out/binaries.sha256"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
run_one(){
 local bench_name="$1" bench_recording="$2";shift 2
 echo "Starting $bench_name"
 set +e
 bin/vins-fusion "$bench_recording" "$bench_out/$bench_name" --opencv-threads 1 "$@" > "$bench_out/$bench_name.log" 2>&1
 local bench_status=$?
 set -e
 printf '%s\n' "$bench_status" > "$bench_out/$bench_name.exit"
 echo "Finished $bench_name (exit $bench_status)"
 [[ -f "$bench_out/$bench_name/summary.json" ]] || return 1
}
run_one two-minute-vins-fusion recordings/diagnostic/20261025T095519.847430960Z
run_one two-minute-vins-fusion-native-noise recordings/diagnostic/20261025T095519.847430960Z --native-noise
run_one eighty-second-vins-fusion recordings/diagnostic/20261025T091926.822317376Z
/usr/sbin/nvpmodel -q > "$bench_out/power-end.txt"
touch "$bench_out/COMPLETE"
