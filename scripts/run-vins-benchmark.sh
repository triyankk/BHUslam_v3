#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$BHU_ROOT"
bench_out="${1:?Usage: run-vins-benchmark.sh NEW_OUTPUT [--both-recordings]}"
bench_both="${2:-}"
[[ ! -e "$bench_out" ]] || { echo 'Output already exists' >&2;exit 1; }
[[ -x build/vins-mono/bin/vins_mono_replay ]] || { echo 'Build VINS first' >&2;exit 1; }
bench_power="$(/usr/sbin/nvpmodel -q)"
[[ "$bench_power" == *'NV Power Mode: 15W'* ]] || { echo "Expected 15W; actual: $bench_power" >&2;exit 1; }
mkdir -p "$bench_out"
printf '%s\n' "$bench_power" > "$bench_out/power-start.txt"
cat /proc/sys/kernel/random/boot_id > "$bench_out/boot-id.txt"
git -C third_party/VINS-Mono rev-parse HEAD > "$bench_out/vins-commit.txt"
git -C third_party/ceres-1.14 rev-parse HEAD > "$bench_out/ceres-commit.txt"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
run_one(){
  local bench_name="$1";shift
  echo "Starting $bench_name"
  set +e
  "$@" > "$bench_out/$bench_name.log" 2>&1
  local bench_status=$?
  set -e
  printf '%s\n' "$bench_status" > "$bench_out/$bench_name.exit"
  echo "Finished $bench_name (exit $bench_status)"
}
bench_dataset(){
  local bench_label="$1" bench_recording="$2" bench_extra="$3"
  cp "$bench_recording/orb.yaml" "$bench_out/$bench_label-orb.yaml"
  sed -i '/^loopClosing:/d' "$bench_out/$bench_label-orb.yaml"
  printf '\nloopClosing: 0\n' >> "$bench_out/$bench_label-orb.yaml"
  run_one "$bench_label-vins" bin/vins-mono "$bench_recording" "$bench_out/$bench_label-vins" --opencv-threads 1
  run_one "$bench_label-orb-mono" bin/bhu replay "$bench_recording" third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt --inertial --monocular --allow-provisional --opencv-threads 1 --settings "$bench_out/$bench_label-orb.yaml" --output "$bench_out/$bench_label-orb-mono"
  if [[ "$bench_extra" == yes ]];then
    run_one "$bench_label-vins-native-noise" bin/vins-mono "$bench_recording" "$bench_out/$bench_label-vins-native-noise" --native-noise --opencv-threads 1
    run_one "$bench_label-orb-stereo" bin/bhu replay "$bench_recording" third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt --inertial --allow-provisional --opencv-threads 1 --settings "$bench_out/$bench_label-orb.yaml" --output "$bench_out/$bench_label-orb-stereo"
  fi
}
bench_dataset two-minute recordings/diagnostic/20261025T095519.847430960Z yes
if [[ "$bench_both" == --both-recordings ]];then
  bench_dataset eighty-second recordings/diagnostic/20261025T091926.822317376Z no
fi
/usr/sbin/nvpmodel -q > "$bench_out/power-end.txt"
sha256sum build/vins-mono/bin/vins_mono_replay build/main/bin/gemini_replay > "$bench_out/binaries.sha256"
touch "$bench_out/COMPLETE"
